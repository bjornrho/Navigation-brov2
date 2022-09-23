import math
import numpy as np
import csv
from numpy.linalg import norm
from datetime import datetime

from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from brov2_interfaces.msg import DVL, Barometer
from brov2_qekf import qekf

from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from bluerov_interfaces.msg import Reference

import sys
sys.path.append('utility_functions')
import utility_functions


class StateEstimateSubPub(Node):

    def __init__(self):
        super().__init__('qekf_state_estimator')
        # Declaring and getting parameters for QEKF
        self.declare_parameters(
            namespace='',
            parameters=[
                ('x_0',[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                ('std_a',                       np.sqrt(4.862*10**-1)),             #datasheet: (190 * 9.81 * 10**-6 * np.sqrt(100))
                ('std_gyro',                    np.sqrt(1.298*10**-4)),             #datasheet: (0.3 * (np.pi/180) * np.sqrt(100))
                ('std_dvl',                     0.01),
                ('std_depth',                   np.sqrt(6.090*10**-6)),
                ('std_orientation',             np.sqrt(8.404*10**-7)),             #datasheet: (3*np.pi/180)
                ('std_a_bias',                  np.sqrt(4.578*10**-2)),             #datasheet: 0.0014
                ('std_gyro_bias',               np.sqrt(2.033*10**-4)),             #datasheet: 0.0038
                ('dvl_offset',                  [-0.020, -0.095, 0.133]),
                ('barometer_offset',            [-0.175, -0.015, -0.05]),
                ('imu_offset',                  [0.057, 0.027, -0.025]),
                ('dvl_vel_topic_name',          'dvl/velocity_estimate'),
                ('imu_topic_name',              'bno055/imu'),
                ('barometer_topic_name',        'barometer/barometer_data'),
                ('state_estimate_topic_name',   '/CSEI/observer/odom')
            ]
        )

        (self.x_0,self.std_a,self.std_gyro,self.std_dvl,self.std_depth,self.std_orientation, self.std_a_bias,
        self.std_gyro_bias,self.dvl_offset,self.barometer_offset,self.imu_offset,dvl_vel_topic_name,imu_topic_name,
        barometer_topic_name,state_estimate_topic_name) = self.get_parameters([ 'x_0','std_a','std_gyro','std_dvl',
                                                                                'std_depth','std_orientation',
                                                                                'std_a_bias','std_gyro_bias',
                                                                                'dvl_offset','barometer_offset',
                                                                                'imu_offset','dvl_vel_topic_name',
                                                                                'imu_topic_name','barometer_topic_name',
                                                                                'state_estimate_topic_name'])
        
        # Initializing nominal state, error-state, covariances and the QEKF
        self.initialized = False
        self.x = self.x_0.value
        self.dx = np.zeros((18,1))
        self.P = np.eye(18)
        self.QEKF = None
        self.q_offset = None

        # Initializing subscribers for sensors
        self.dvl_subscription = self.create_subscription(DVL, dvl_vel_topic_name.value, self.dvl_vel_sub, 5)
        self.imu_subscription = self.create_subscription(Imu, imu_topic_name.value, self.imu_sub, 10)
        self.barometer_subscription = self.create_subscription(Barometer, barometer_topic_name.value, self.barometer_sub, 5)

        # Initialization of state estimate publisher
        self.state_estimate_publisher = self.create_publisher(Odometry, state_estimate_topic_name.value, 10)
        
        # Initializing current and previous message variables
        self.current_imu = Imu()
        self.current_barometer = Barometer()
        self.current_vel = DVL()      
        self.current_state_estimate = Odometry()

        # Initializing orientation offset parameters
        self.declare_parameter('orientation_offset', value=[1.0, 0.0, 0.0, 0.0])
        self.srv_orientation = self.create_service(Trigger, 'brov2_qekf/set_yaw_offset', self.srv_set_yaw_offset)
        self.srv_filter_reset = self.create_service(Trigger, 'brov2_qekf/reset_qekf', self.srv_reset_qekf)
        self.srv_NIS_logging = self.create_service(Trigger, 'brov2_qekf/start_stop_NIS_logging', self.start_stop_NIS_logging)


    ### Services
    def srv_set_yaw_offset(self, request, response):
        # Inverse of the current corrected orientation is the same as the complex conjugate ([w,-u])
        offset_ned = utility_functions.ENU_to_NED_conversion(np.array([[self.current_imu.orientation.w],
                                                                       [self.current_imu.orientation.x],
                                                                       [self.current_imu.orientation.y],
                                                                       [self.current_imu.orientation.z]]))
        yaw_offset = utility_functions.yaw_from_quaternion(offset_ned)
        q_yaw = np.array([[np.cos(yaw_offset/2)],[0.0],[0.0],[np.sin(yaw_offset/2)]])
        q_yaw /= norm(q_yaw)
        q_yaw[1:] = -q_yaw[1:]

        orientation_offset = Parameter('orientation_offset', Parameter.Type.DOUBLE_ARRAY, (q_yaw.T[0]).tolist())

        self.set_parameters([orientation_offset])
        self.get_logger().info('Set orientation offset: %s' % str(orientation_offset.value))

        response.success = True
        response.message = 'Set orientation offset: %s' % str(orientation_offset.value)

        return response


    def srv_reset_qekf(self, request, response):
        # Resetting filter, but keeping the previous depth measurement
        x_0_init = self.x_0.value
        x_0_init[2] = self.current_barometer.depth
        
        self.QEKF.filter_reset(x_0_init, np.zeros((18,1)), np.eye(18),self.std_a.value, 
                        self.std_gyro.value, self.std_dvl.value, self.std_depth.value, self.std_orientation.value, 
                        self.std_a_bias.value, self.std_gyro_bias.value)

        self.get_logger().info('Filter is reset to initialization values')

        response.success = True
        response.message = 'Filter is reset to initialization values'

        return response

    def start_stop_NIS_logging(self, request, response):
        self.QEKF.store_NIS = not self.QEKF.store_NIS
        if self.QEKF.store_NIS:
            file_directory = "src/brov2_qekf/NIS_values/"
            self.QEKF.file_name = "NIS_" + datetime.now().strftime("%Y%m%d-%H%M%S" + ".csv")
            self.NIS_file = open(file_directory + self.QEKF.file_name, "a+")
            self.QEKF.NIS_writer = csv.writer(self.NIS_file)
            self.QEKF.NIS_writer.writerow([np.nan, np.nan, np.nan,
                                           self.std_gyro.value, self.std_a.value, self.std_depth.value, 
                                           self.std_gyro_bias.value, self.std_a_bias.value])
            self.get_logger().info('Start logging NIS to file \'%s\'' % self.QEKF.file_name)
            response.message = 'NIS logging started.'
        else:
            self.NIS_file.close()
            self.get_logger().info('Stop logging NIS to file \'%s\'' % self.QEKF.file_name)
            response.message = 'NIS logging stopped.'

        response.success = True
        
        return response
    

    def imu_sub(self, msg):
        # Getting stamp of previous imu message and storing the current message
        previous_stamp = self.current_imu.header.stamp
        
        self.current_imu = msg
        
        # Orientation measurements from BNO055 comes in ENU and must be converted to NED
        q_ned = utility_functions.ENU_to_NED_conversion(np.array([[msg.orientation.w],[msg.orientation.x],
                                                                  [msg.orientation.y],[msg.orientation.z]]))
        if self.initialized:
            # Fetching dt
            t_2 = self.current_imu.header.stamp.sec + self.current_imu.header.stamp.nanosec*(10**(-9))
            t_1 = previous_stamp.sec + previous_stamp.nanosec*(10**(-9))
            if t_1 == 0.0:
                return
            dt = t_2 - t_1

            # Fetching u
            a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            omega = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            u = np.block([[a],[omega]])

            # Integrate measurement to form nominal state and propagate error-state and covariances
            self.x = self.QEKF.integrate(u, dt)
            self.dx, self.P = self.QEKF.predict(u, dt)

            # Update orientation with bno055 estimate - making up for stored offset values
            self.q_offset = self.get_parameter('orientation_offset').get_parameter_value().double_array_value

            q = utility_functions.quaternion_product(np.array([self.q_offset]).T, q_ned)
            self.dx, self.P = self.QEKF.update_orientation(q)

            # Injecting observed error-state into nominal state 
            self.x = self.QEKF.inject()

            # Reset of error-state and covariances
            self.dx, self.P = self.QEKF.reset()

            # Prepare odometry message
            self.current_state_estimate.header.stamp = self.get_clock().now().to_msg()
            # Position
            self.current_state_estimate.pose.pose.position = Point(x=self.x[0][0], y=self.x[1][0], z=self.x[2][0])
            # Orientation
            self.current_state_estimate.pose.pose.orientation = Quaternion(w=self.x[6][0], x=self.x[7][0],
                                                                           y=self.x[8][0], z=self.x[9][0])
            # Linear acceleration
            self.current_state_estimate.twist.twist.linear = self.current_imu.linear_acceleration
            # Angular velocity
            self.current_state_estimate.twist.twist.angular = self.current_imu.angular_velocity

            # Covariance values
            pose_covariance = np.zeros((6,6))
            pose_covariance[0:3,0:3] = self.P[0:3,0:3]
            pose_covariance[0:3,3:6] = self.P[0:3,6:9]
            pose_covariance[3:6,0:3] = self.P[6:9,0:3]
            pose_covariance[3:6,3:6] = self.P[6:9,6:9]
            self.current_state_estimate.pose.covariance = pose_covariance.flatten()

            self.state_estimate_publisher.publish(self.current_state_estimate)
        else:        
            # Initializing QEKF
            self.QEKF = qekf.QEKF(self.x, self.dx, self.P, self.std_a.value, self.std_gyro.value, self.std_dvl.value, 
                                  self.std_depth.value, self.std_orientation.value, self.std_a_bias.value, 
                                  self.std_gyro_bias.value, self.dvl_offset.value, self.barometer_offset.value, 
                                  self.imu_offset.value)

            # Setting offset such that the filter gets initialized at [1.0, 0.0, 0.0, 0.0]
            yaw_offset = utility_functions.yaw_from_quaternion(q_ned)
            q_yaw = np.array([[np.cos(yaw_offset/2)],[0.0],[0.0],[np.sin(yaw_offset/2)]])
            q_yaw /= norm(q_yaw)
            q_yaw[1:] = -q_yaw[1:]


            orientation_offset = Parameter('orientation_offset', Parameter.Type.DOUBLE_ARRAY, (q_yaw.T[0]).tolist())

            self.set_parameters([orientation_offset])
            
            self.initialized = True
            self.get_logger().info('Initialization of QEKF and topics are finished.')



    def barometer_sub(self, msg):
        if self.initialized:
            # Storing the current message and fetching depth measurement
            self.current_barometer = msg
            depth_measurement = self.current_barometer.depth

            # Updating QEKF with depth measurement
            self.dx, self.P = self.QEKF.update_depth(depth_measurement)

            # Injecting observed error-state into nominal state 
            self.x = self.QEKF.inject()

            # Reset of error-state and covariances
            self.dx, self.P = self.QEKF.reset()

            # Prepare odometry message
            self.current_state_estimate.header.stamp = self.get_clock().now().to_msg()
            # Position
            self.current_state_estimate.pose.pose.position = Point(x=self.x[0][0], y=self.x[1][0], z=self.x[2][0])
            # Orientation
            self.current_state_estimate.pose.pose.orientation = Quaternion(w=self.x[6][0], x=self.x[7][0],
                                                                           y=self.x[8][0], z=self.x[9][0])
            # Linear acceleration
            self.current_state_estimate.twist.twist.linear = self.current_imu.linear_acceleration
            # Angular velocity
            self.current_state_estimate.twist.twist.angular = self.current_imu.angular_velocity

            # Covariance values
            pose_covariance = np.zeros((6,6))
            pose_covariance[0:3,0:3] = self.P[0:3,0:3]
            pose_covariance[0:3,3:6] = self.P[0:3,6:9]
            pose_covariance[3:6,0:3] = self.P[6:9,0:3]
            pose_covariance[3:6,3:6] = self.P[6:9,6:9]
            self.current_state_estimate.pose.covariance = pose_covariance.flatten()


            self.state_estimate_publisher.publish(self.current_state_estimate)
            


    def dvl_vel_sub(self, msg):
        if self.initialized:
            # Storing the current message and fetching velocity measurements
            self.current_vel = msg
            dvl_measurement = np.array([[self.current_vel.velocity.x],
                                        [self.current_vel.velocity.y],
                                        [self.current_vel.velocity.z]])

            dvl_covariance = msg.covariance.reshape((3,3))

            if self.current_vel.velocity_valid:
                # Updating QEKF with DVL measurement 
                self.dx, self.P = self.QEKF.update_dvl(dvl_measurement, dvl_covariance)

                # Injecting observed error-state into nominal state 
                self.x = self.QEKF.inject()

                # Reset of error-state and covariances
                self.dx, self.P = self.QEKF.reset()

                # Prepare odometry message
                self.current_state_estimate.header.stamp = self.get_clock().now().to_msg()
                # Position
                self.current_state_estimate.pose.pose.position = Point(x=self.x[0][0], y=self.x[1][0], z=self.x[2][0])
                # Orientation
                self.current_state_estimate.pose.pose.orientation = Quaternion(w=self.x[6][0], x=self.x[7][0],
                                                                               y=self.x[8][0], z=self.x[9][0])
                # Linear acceleration
                self.current_state_estimate.twist.twist.linear = self.current_imu.linear_acceleration
                # Angular velocity
                self.current_state_estimate.twist.twist.angular = self.current_imu.angular_velocity

                # Covariance values
                pose_covariance = np.zeros((6,6))
                pose_covariance[0:3,0:3] = self.P[0:3,0:3]
                pose_covariance[0:3,3:6] = self.P[0:3,6:9]
                pose_covariance[3:6,0:3] = self.P[6:9,0:3]
                pose_covariance[3:6,3:6] = self.P[6:9,6:9]
                self.current_state_estimate.pose.covariance = pose_covariance.flatten()


                self.state_estimate_publisher.publish(self.current_state_estimate)