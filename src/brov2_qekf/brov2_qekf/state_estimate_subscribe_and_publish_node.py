import numpy as np

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
                ('x_0',[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81]),
                ('std_a', np.sqrt(0.017)),
                ('std_gyro', np.sqrt(0.04)),
                ('std_dvl', 0.2626),
                ('std_depth', 0.255),
                ('std_a_bias', 0.0014),
                ('std_gyro_bias', 0.0038),
                ('dvl_offset', [0.0,0.0,0.0]),#[-0.020, -0.100, 0.115]),
                ('barometer_offset', [0.0,0.0,0.0]),#[-0.175, -0.015, -0.05]),
                ('imu_offset', [0.0,0.0,0.0]),#[0.055, 0.033, -0.03]),
            ]
        )

        (x_0,self.std_a,self.std_gyro,self.std_dvl,self.std_depth,self.std_a_bias,self.std_gyro_bias,
        self.dvl_offset,self.barometer_offset,self.imu_offset) = self.get_parameters(['x_0','std_a',
                                                                                                         'std_gyro',
                                                                                                         'std_dvl',
                                                                                                         'std_depth',
                                                                                                         'std_a_bias',
                                                                                                         'std_gyro_bias',
                                                                                                         'dvl_offset',
                                                                                                         'barometer_offset',
                                                                                                         'imu_offset'])
        
        # Initializing nominal state, error-state, covariances and the QEKF
        self.initialized = False
        self.x = x_0.value
        self.dx = np.zeros((18,1))
        self.P = np.eye(18)
        self.QEKF = None

        # Initializing subscribers for sensors
        self.dvl_subscription = self.create_subscription(DVL, 'velocity_estimate', self.dvl_vel_sub, 10)
        self.imu_subscription = self.create_subscription(Imu, 'bno055/imu', self.imu_sub, 10)
        self.barometer_subscription = self.create_subscription(Barometer, 'barometer/barometer_data', self.barometer_sub, 10)

        # Initialization of state estimate publisher
        self.state_estimate_publisher = self.create_publisher(Odometry, '/CSEI/observer/odom', 10)
        
        # Initializing current and previous message variables
        self.current_imu = Imu()
        self.current_barometer = Barometer()
        self.current_vel = DVL()      
        self.current_state_estimate = Odometry()

        # Initializing orientation offset parameters
        self.declare_parameter('orientation_offset', value=[1.0, 0.0, 0.0, 0.0])
        self.srv_orientation = self.create_service(Trigger, 'brov2_qekf/set_orient_offset',self.srv_set_orientation_offset)

    def srv_set_orientation_offset(self, request, response):
        # Inverse of the current corrected orientation is the same as the complex conjugate ([w,-u])
        orientation_offset = Parameter('orientation_offset', Parameter.Type.DOUBLE_ARRAY, 
                                        [self.current_imu.orientation.w, -self.current_imu.orientation.x,
                                        -self.current_imu.orientation.y, -self.current_imu.orientation.z])

        self.set_parameters([orientation_offset])
        print(orientation_offset.value)
        self.get_logger().info('Set orientation offset: %s' % str(orientation_offset.value))

        response.success = True
        response.message = 'Set orientation offset: %s' % str(orientation_offset.value)

        return response

    def imu_sub(self, msg):
        if self.initialized:
            # Getting stamp of previous imu message and storing the current message
            previous_stamp = self.current_imu.header.stamp
            self.current_imu = msg
            # Fetching dt
            #t_2 = self.current_imu.header.stamp.sec + self.current_imu.header.stamp.nanosec*(10**(-9))
            #t_1 = previous_stamp.sec + previous_stamp.nanosec*(10**(-9))
            #if t_1 == 0.0:
            #    return
            dt = 1 #t_2 - t_1

            # Fetching u
            a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            omega = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            u = np.block([[a],[omega]])

            # Integrate measurement to form nominal state and propagate error-state and covariances
            self.x = self.QEKF.integrate(u, dt)
            self.dx, self.P = self.QEKF.predict(u, dt)

            # Update orientation with bno055 estimate - making up for stored offset values
            q = np.array([[msg.orientation.w],[msg.orientation.x],[msg.orientation.y],[msg.orientation.z]])
            q_offset = self.get_parameter('orientation_offset').get_parameter_value().double_array_value

            q = self.QEKF.quaternion_product(q, np.array([q_offset]).T)
            print("Offset: \n", q)
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
            # Initializing with the current bno055 orientation estimate
            self.x[6:10] = np.array([[msg.orientation.w],[msg.orientation.x],
                                     [msg.orientation.y],[msg.orientation.z]]).T[0]
            self.QEKF = qekf.QEKF(self.x, self.dx, self.P, self.std_a.value, self.std_gyro.value, self.std_dvl.value, 
                                  self.std_depth.value, self.std_a_bias.value, self.std_gyro_bias.value, 
                                  self.dvl_offset.value, self.barometer_offset.value, self.imu_offset.value)
            self.initialized = True
            self.get_logger().info('Initialization of QEKF and topics are finished.')



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
        # Storing the current message and fetching velocity measurements
        self.current_vel = msg
        dvl_measurement = np.array([[self.current_vel.velocity.x],
                                    [self.current_vel.velocity.y],
                                    [self.current_vel.velocity.z]])

        dvl_covariance = msg.covariance.reshape((3,3))
        
        if True: #self.current_vel.velocity_valid:
            # Updating QEKF with DVL measurement 
            # (using only x- and y-values since the flat-floor assumption is not necessarily valid)
            self.dx, self.P = self.QEKF.update_dvl(dvl_measurement[:2], dvl_covariance[:2,:2])

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


# -2 -10 11.5 trekke fra

