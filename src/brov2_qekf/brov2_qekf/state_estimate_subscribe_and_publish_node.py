import math
import numpy as np
from numpy.linalg import norm

from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from brov2_interfaces.msg import DVL, Barometer
from brov2_qekf import qekf

from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from bluerov_interfaces.msg import Reference


class StateEstimateSubPub(Node):

    def __init__(self):
        super().__init__('state_estimate_sub_pub')
        # Declaring and getting parameters for QEKF
        self.declare_parameters(
            namespace='',
            parameters=[
                ('x_0',[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                ('std_a',               4.862*10**-1),                  #datasheet: (190 * 9.81 * 10**-6 * np.sqrt(100))
                ('std_gyro',            1.298*10**-4),                  #datasheet: (0.3 * (np.pi/180) * np.sqrt(100))
                ('std_dvl',             0.01),
                ('std_depth',           6.090*10**-6),
                ('std_orientation',     8.404*10**-7),                  #datasheet: (3*np.pi/180)
                ('std_a_bias',          4.578*10**-2),                  #datasheet: 0.0014
                ('std_gyro_bias',       2.033*10**-4),                  #datasheet: 0.0038
                ('dvl_offset',          [-0.020, -0.095, 0.133]),
                ('barometer_offset',    [-0.175, -0.015, -0.05]),
                ('imu_offset',          [0.057, 0.027, -0.025]),
                ('store_NIS',           False),
            ]
        )

        (self.x_0,self.std_a,self.std_gyro,self.std_dvl,self.std_depth,self.std_orientation, self.std_a_bias,
        self.std_gyro_bias,self.dvl_offset,self.barometer_offset,self.imu_offset,self.store_NIS) = self.get_parameters(['x_0','std_a',
                                                                                                         'std_gyro',
                                                                                                         'std_dvl',
                                                                                                         'std_depth',
                                                                                                         'std_orientation',
                                                                                                         'std_a_bias',
                                                                                                         'std_gyro_bias',
                                                                                                         'dvl_offset',
                                                                                                         'barometer_offset',
                                                                                                         'imu_offset',
                                                                                                         'store_NIS'])
        
        # Initializing nominal state, error-state, covariances and the QEKF
        self.initialized = False
        self.x = self.x_0.value
        self.dx = np.zeros((18,1))
        self.P = np.eye(18)
        self.QEKF = None
        self.q_offset = None

        # Initializing subscribers for sensors
        self.dvl_subscription = self.create_subscription(DVL, 'dvl/velocity_estimate', self.dvl_vel_sub, 5)
        self.imu_subscription = self.create_subscription(Imu, 'bno055/imu', self.imu_sub, 10)
        self.barometer_subscription = self.create_subscription(Barometer, 'barometer/barometer_data', self.barometer_sub, 5)

        # Initialization of state estimate publisher
        self.state_estimate_publisher = self.create_publisher(Odometry, '/CSEI/observer/odom', 10)
        
        # Initializing current and previous message variables
        self.current_imu = Imu()
        self.current_barometer = Barometer()
        self.current_vel = DVL()      
        self.current_state_estimate = Odometry()

        # Initializing orientation offset parameters
        self.declare_parameter('orientation_offset', value=[1.0, 0.0, 0.0, 0.0])
        self.srv_orientation = self.create_service(Trigger, 'brov2_qekf/set_orient_offset',self.srv_set_yaw_offset)
        self.srv_filter_reset = self.create_service(Trigger, 'brov2_qekf/reset_qekf',self.srv_reset_qekf)

        # Initializing running variance estimation variables
        self.variance_initialized = False
        self.previous_measurement = None
        self.M = None
        self.S = None
        self.k = 1


    ### Services
    def srv_set_yaw_offset(self, request, response):
        # Inverse of the current corrected orientation is the same as the complex conjugate ([w,-u])
        offset_ned = self.ENU_to_NED_conversion(np.array([[self.current_imu.orientation.w],
                                                          [self.current_imu.orientation.x],
                                                          [self.current_imu.orientation.y],
                                                          [self.current_imu.orientation.z]]))
        yaw_offset = self.yaw_from_quaternion(offset_ned)
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
    

    def imu_sub(self, msg):
        # Getting stamp of previous imu message and storing the current message
        previous_stamp = self.current_imu.header.stamp

        ## RELATED TO RUNNING COVARIANCE COMPUTATION
        #self.previous_measurement = np.array([[self.current_imu.linear_acceleration.x],
        #                                      [self.current_imu.linear_acceleration.y],
        #                                      [self.current_imu.linear_acceleration.z]])
        
        self.current_imu = msg
        
        # Orientation measurements from BNO055 comes in ENU and must be converted to NED
        q_ned = self.ENU_to_NED_conversion(np.array([[msg.orientation.w],[msg.orientation.x],
                                                    [msg.orientation.y],[msg.orientation.z]]))
        #q_ned = np.array([[msg.orientation.w],[msg.orientation.x],[msg.orientation.y],[msg.orientation.z]])
        if self.initialized:
            self.k += 1
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

            q = self.QEKF.quaternion_product(np.array([self.q_offset]).T, q_ned)
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
    
            self.state_estimate_publisher.publish(self.current_state_estimate)
        else:        
            # Initializing QEKF
            self.QEKF = qekf.QEKF(self.x, self.dx, self.P, self.std_a.value, self.std_gyro.value, self.std_dvl.value, 
                                  self.std_depth.value, self.std_orientation.value, self.std_a_bias.value, 
                                  self.std_gyro_bias.value, self.dvl_offset.value, self.barometer_offset.value, 
                                  self.imu_offset.value, self.store_NIS.value)

            # Setting offset such that the filter gets initialized at [1.0, 0.0, 0.0, 0.0]
            yaw_offset = self.yaw_from_quaternion(q_ned)
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

            #if not self.variance_initialized:
            ## Defining variables for running variance computation
            #    self.M = self.current_barometer.depth
            #    self.S = 0
            #    self.variance_initialized = True
            #print(self.computing_running_variance(depth_measurement))

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
                #print("DVL measurement: \n", dvl_measurement)
                #print("DVL measurement oriented: \n", (R.T@dvl_measurement)[:2])
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

                self.state_estimate_publisher.publish(self.current_state_estimate)

    def computing_running_variance(self, measurement):
        prev_M = self.M
        self.M = self.M + (measurement - self.M) / self.k
        self.S = self.S + (measurement - prev_M)*(measurement - self.M)
        
        return self.S/(self.k-1)


    @staticmethod
    def ENU_to_NED_conversion(quaternion):
        """Converts given quaternion from ENU to NED.

        Args:
            quaternion       (4,1 ndarray) : Quaternion in ENU of form [w,x,y,z]


        Returns:
            qproduct         (4,1 ndarray) : Normalized quaternion in NED
            """
        p_w,p_x,p_y,p_z = [0.0, -np.sqrt(1/2), -np.sqrt(1/2), 0.0]
        q_w,q_x,q_y,q_z = quaternion.T[0]
        qproduct = np.array([[p_w*q_w - p_x*q_x - p_y*q_y - p_z*q_z],
                             [p_w*q_x + p_x*q_w + p_y*q_z - p_z*q_y],
                             [p_w*q_y - p_x*q_z + p_y*q_w + p_z*q_x],
                             [p_w*q_z + p_x*q_y - p_y*q_x + p_z*q_w]])

        return qproduct/norm(qproduct) 

    @staticmethod
    def yaw_from_quaternion(quaternion):
        """Returns yaw (Euler angle - rotation around z counterclockwise) in radians.

        Args:
            quaternion       (4,1 ndarray) : Quaternion of form [w,x,y,z]


        Returns:
            yaw_z            (4,1 ndarray) : Normalized quaternion
            """

        q_w,q_x,q_y,q_z = quaternion.T[0]

        t0 = +2.0 * (q_w * q_z + q_x * q_y)
        t1 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        yaw_z = math.atan2(t0, t1)

        return yaw_z



# -2 -10 11.5 trekke fra

