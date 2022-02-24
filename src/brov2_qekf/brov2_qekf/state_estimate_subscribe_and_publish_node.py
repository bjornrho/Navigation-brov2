from numpy.core.numeric import roll
from rosidl_parser.definition import Include
from scipy.spatial.transform import Rotation
import quaternion

import math
import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from brov2_interfaces.msg import DVL, Barometer
from brov2_qekf import qekf


class StateEstimateSubPub(Node):

    def __init__(self):
        super().__init__('state_estimate_sub_pub')
        # Declaring and getting parameters for QEKF
        self.declare_parameters(
            namespace='',
            parameters=[
                ('x_0',[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -9.81]),
                ('std_a', np.sqrt(0.017)),
                ('std_gyro', np.sqrt(0.04)),
                ('std_dvl', 1.0),
                ('std_depth', 0.255),
                ('std_a_bias', 0.0014),
                ('std_gyro_bias', 0.0038),
                ('dvl_offset', [0.0, 0.0, 0.0])
            ]
        )

        (x_0,std_a,std_gyro,std_dvl,std_depth,std_a_bias,std_gyro_bias,dvl_offset) = self.get_parameters(['x_0','std_a',
                                                                                                         'std_gyro',
                                                                                                         'std_dvl',
                                                                                                         'std_depth',
                                                                                                         'std_a_bias',
                                                                                                         'std_gyro_bias',
                                                                                                         'dvl_offset'])
        
        # Initializing nominal state, error-state, covariances and the QEKF
        self.x = x_0.value
        self.dx = np.zeros((18,1))
        self.P = np.eye(18)
        self.QEKF = qekf.QEKF(self.x, self.dx, self.P, std_a.value, std_gyro.value, std_dvl.value, std_depth.value, 
                                std_a_bias.value, std_gyro_bias.value, dvl_offset.value)

        # Initializing subscribers for sensors
        self.dvl_subscription = self.create_subscription(DVL, 'dvl/velocity_estimate', self.dvl_vel_sub, 10)
        self.imu_subscription = self.create_subscription(Imu, 'bno055/imu_raw', self.imu_sub, 10)
        self.barometer_subscription = self.create_subscription(Barometer, 'barometer/barometer_data', self.barometer_sub, 10)

        # Initialization of state estimate publisher
        self.state_estimate_publisher = self.create_publisher(Odometry, '/CSEI/observer/odom', 10)
        
        # Initializing current and previous message variables
        self.current_imu = Imu()
        self.current_barometer = Barometer()
        self.current_vel = DVL()      
        self.current_state_estimate = Odometry()


    def imu_sub(self, msg):
        # Getting stamp of previous imu message and storing the current message
        previous_stamp = self.current_imu.header.stamp
        self.current_imu = msg
        
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

        # Prepare odometry message
        self.current_state_estimate.header.stamp = self.get_clock().now().to_msg()
        # Position
        self.current_state_estimate.pose.pose.position.x = self.x[0][0]
        self.current_state_estimate.pose.pose.position.y = self.x[1][0]
        self.current_state_estimate.pose.pose.position.z = self.x[2][0]
        # Orientation
        self.current_state_estimate.pose.pose.orientation.w = self.x[6][0]
        self.current_state_estimate.pose.pose.orientation.x = self.x[7][0]
        self.current_state_estimate.pose.pose.orientation.y = self.x[8][0]
        self.current_state_estimate.pose.pose.orientation.z = self.x[9][0]
        # Linear acceleration
        self.current_state_estimate.twist.twist.linear = self.current_imu.linear_acceleration
        # Angular velocity
        self.current_state_estimate.twist.twist.angular = self.current_imu.angular_velocity

        self.state_estimate_publisher.publish(self.current_state_estimate)

    def barometer_sub(self, msg):
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
        self.current_state_estimate.pose.pose.position.x = self.x[0][0]
        self.current_state_estimate.pose.pose.position.y = self.x[1][0]
        self.current_state_estimate.pose.pose.position.z = self.x[2][0]
        # Orientation
        self.current_state_estimate.pose.pose.orientation.w = self.x[6][0]
        self.current_state_estimate.pose.pose.orientation.x = self.x[7][0]
        self.current_state_estimate.pose.pose.orientation.y = self.x[8][0]
        self.current_state_estimate.pose.pose.orientation.z = self.x[9][0]
        # Linear acceleration
        self.current_state_estimate.twist.twist.linear = self.current_imu.linear_acceleration
        # Angular velocity
        self.current_state_estimate.twist.twist.angular = self.current_imu.angular_velocity

        self.state_estimate_publisher.publish(self.current_state_estimate)


    def dvl_vel_sub(self, msg):
        # Storing the current message and fetching velocity measurements
        self.current_vel = msg
        dvl_measurement = np.array([[self.current_vel.velocity.x],
                                    [self.current_vel.velocity.y],
                                    [self.current_vel.velocity.z]])
        
        if self.current_vel.velocity_valid:
            # Updating QEKF with DVL measurement
            self.dx, self.P = self.QEKF.update_dvl(dvl_measurement)

            # Injecting observed error-state into nominal state 
            self.x = self.QEKF.inject()

            # Reset of error-state and covariances
            self.dx, self.P = self.QEKF.reset()

            # Prepare odometry message
            self.current_state_estimate.header.stamp = self.get_clock().now().to_msg()
            # Position
            self.current_state_estimate.pose.pose.position.x = self.x[0][0]
            self.current_state_estimate.pose.pose.position.y = self.x[1][0]
            self.current_state_estimate.pose.pose.position.z = self.x[2][0]
            # Orientation
            self.current_state_estimate.pose.pose.orientation.w = self.x[6][0]
            self.current_state_estimate.pose.pose.orientation.x = self.x[7][0]
            self.current_state_estimate.pose.pose.orientation.y = self.x[8][0]
            self.current_state_estimate.pose.pose.orientation.z = self.x[9][0]
            # Linear acceleration
            self.current_state_estimate.twist.twist.linear = self.current_imu.linear_acceleration
            # Angular velocity
            self.current_state_estimate.twist.twist.angular = self.current_imu.angular_velocity

            self.state_estimate_publisher.publish(self.current_state_estimate)


# -2 -10 11.5 trekke fra