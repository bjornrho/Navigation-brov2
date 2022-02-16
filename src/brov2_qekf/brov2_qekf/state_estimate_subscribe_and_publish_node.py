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
                ('std_a', 0.0),
                ('std_gyro', 0.0),
                ('std_dvl', 0.0),
                ('std_depth', 1.0),
                ('std_a_bias', 1.0),
                ('std_gyro_bias', 1.0),
                ('dvl_offset', [2.0, 2.0, 2.0])
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
        self.imu_subscription = self.create_subscription(Imu, 'bno055/imu/data', self.imu_sub, 10)
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
        self.x = self.QEKF.inject(dt) # WHAT dt??

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
            self.x = self.QEKF.inject(dt) # WHAT dt??

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



    '''
    def set_position_offset(self, request, response):
        position_offset_x = Parameter('position_offset_x', Parameter.Type.DOUBLE, self.current_odom.x)
        position_offset_y = Parameter('position_offset_y', Parameter.Type.DOUBLE, self.current_odom.y)
        self.set_parameters([position_offset_x, position_offset_y])
        self.get_logger().info('Set position offset\nx: %d y: %d' % (self.current_odom.x, self.current_odom.y))
        
        response.success = True
        response.message = 'Set position offset x: %f y: %f' % (self.current_odom.x, self.current_odom.y)

        return response

    def set_orientation_offset(self, request, response):
        # Inverse of the current corrected orientation is the same as the complex conjugate 
        conjugated_orientation = self.current_corrected_orientation.conjugate()

        w_offset, x_offset, y_offset, z_offset = quaternion.as_float_array(conjugated_orientation)
        
        orientation_offset_w = Parameter('orientation_offset_w', Parameter.Type.DOUBLE, w_offset)
        orientation_offset_x = Parameter('orientation_offset_x', Parameter.Type.DOUBLE, x_offset)
        orientation_offset_y = Parameter('orientation_offset_y', Parameter.Type.DOUBLE, y_offset)
        orientation_offset_z = Parameter('orientation_offset_z', Parameter.Type.DOUBLE, z_offset)

        self.set_parameters([orientation_offset_w, orientation_offset_x, orientation_offset_y, orientation_offset_z])
        self.get_logger().info('Set orientation offset w: %f x: %f y: %f z: %f' % (w_offset, x_offset, y_offset, z_offset))

        response.success = True
        response.message = 'Set orientation offset w: %f    x: %f y: %f z: %f' % (w_offset, x_offset, y_offset, z_offset)

        return response

    
    def state_estimate_publisher(self):
        self.current_state_estimate.header.stamp = self.get_clock().now().to_msg()
        # Odometry offset variables
        position_offset_x = self.get_parameter('position_offset_x').get_parameter_value().double_value
        position_offset_y = self.get_parameter('position_offset_y').get_parameter_value().double_value
        orientation_offset_w = self.get_parameter('orientation_offset_w').get_parameter_value().double_value
        orientation_offset_x = self.get_parameter('orientation_offset_x').get_parameter_value().double_value
        orientation_offset_y = self.get_parameter('orientation_offset_y').get_parameter_value().double_value
        orientation_offset_z = self.get_parameter('orientation_offset_z').get_parameter_value().double_value
        orientation_offset_quat = np.quaternion(orientation_offset_w,
                                                orientation_offset_x,
                                                orientation_offset_y, 
                                                orientation_offset_z)

        # Previous barometer message
        self.previous_barometer = self.current_state_estimate.pose.pose.position.z
        
        
        # IMU axis correction - from ENU to NED (IMU orientation initialized in ros2_bno055_sensor)
        quaternion_imu_raw = np.quaternion(self.current_imu.orientation.w,
                                   self.current_imu.orientation.x,
                                   self.current_imu.orientation.y,
                                   self.current_imu.orientation.z)

        quaternion_rot_correction = np.quaternion(0, -np.sqrt(2)/2, -np.sqrt(2)/2, 0) 
        # Erlends suggestion: np.quaternion(0, np.sqrt(2)/2, np.sqrt(2)/2, 0)
        
        self.current_corrected_orientation = quaternion_rot_correction * quaternion_imu_raw # Correct order?

        # Orientation

        ##### DVL #####
        rot = Rotation.from_euler('xyz', [self.current_odom.roll, self.current_odom.pitch, self.current_odom.yaw],degrees=True)
        quat = rot.as_quat()
        
        # self.current_state_estimate.pose.pose.orientation.w = quat[3]
        # self.current_state_estimate.pose.pose.orientation.x = quat[0]
        # self.current_state_estimate.pose.pose.orientation.y = quat[1]
        # self.current_state_estimate.pose.pose.orientation.z = quat[2]
        ###############

        ##### IMU #####
        w, x, y, z = quaternion.as_float_array(self.current_corrected_orientation * orientation_offset_quat)
        self.current_state_estimate.pose.pose.orientation.w = w
        self.current_state_estimate.pose.pose.orientation.x = x
        self.current_state_estimate.pose.pose.orientation.y = y
        self.current_state_estimate.pose.pose.orientation.z = z
        ###############

        dvl_placement_offset = rot.as_matrix()@np.array([-0.020, -0.100, 0.115])
        


        

        # Position

        ###### DVL ######
        self.current_state_estimate.pose.pose.position.x = self.current_odom.x - position_offset_x - dvl_placement_offset[0]
        self.current_state_estimate.pose.pose.position.y = self.current_odom.y - position_offset_y - dvl_placement_offset[1]
        #################

        ###### Homemade odometry ######
        # u = np.array([x,y,z])
        # v = np.array([self.current_vel.velocity.x, self.current_vel.velocity.y, self.current_state_estimate.twist.twist.linear.z])
        # 
        # vel_vector = 2.0 * np.dot(u, v) * u + (w*w - np.dot(u,u)) * v + 2.0 * w*np.cross(u,v) #FIXED *v instead of *u second term!!
        # 
        # self.current_state_estimate.pose.pose.position.x = self.current_state_estimate.pose.pose.position.x + vel_vector[0]*0.1
        # self.current_state_estimate.pose.pose.position.y = self.current_state_estimate.pose.pose.position.y + vel_vector[1]*0.1
        ###############################    

        self.current_state_estimate.pose.pose.position.z = self.current_barometer.depth


        # Velocity angular
        self.current_state_estimate.twist.twist.angular.x = self.current_imu.angular_velocity.x
        self.current_state_estimate.twist.twist.angular.y = self.current_imu.angular_velocity.y
        self.current_state_estimate.twist.twist.angular.z = self.current_imu.angular_velocity.z
        vel_ang_vec = np.array([self.current_imu.angular_velocity.x,
                                self.current_imu.angular_velocity.y,
                                self.current_imu.angular_velocity.z])

        # Velocity linear - z should be barometer derivated and low pass filtered
        self.current_state_estimate.twist.twist.linear.x = self.current_vel.velocity.x - np.cross(vel_ang_vec, dvl_placement_offset)[0]
        self.current_state_estimate.twist.twist.linear.y = self.current_vel.velocity.y - np.cross(vel_ang_vec, dvl_placement_offset)[1]
        #self.current_state_estimate.twist.twist.linear.z = self.current_vel.velocity.z
        self.current_state_estimate.twist.twist.linear.z = (self.current_state_estimate.pose.pose.position.z - self.previous_barometer) / 0.1

        self.state_estimate_publisher.publish(self.current_state_estimate)

        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(w,x,y,z)#quat[3],quat[0],quat[1],quat[2])

        print("Converted quaternion to euler angles: ")
        print("Roll around x:  ", roll_x)
        print("Pitch around y: ", pitch_y)
        print("Yaw around z:   ", yaw_z)
        print("--------------------------------------")
        


    @staticmethod
    def euler_from_quaternion(self, w, x, y, z):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

        A = np.block([[I,       I*dt,   np.diag(((1/2)*R_q@(a_m - a_b)*(dt**2)).T[0]),      zero,   zero,   (1/2)*I*(dt**2)],
                      [zero,    I,      np.diag((R_q@(a_m - a_b)*dt).T[0]),                 zero,   zero,   I*dt],
                      [zero,    zero,   np.diag(self.quaternion_product(q,q_rot).T[0]),     zero,   zero,   zero],
                      [zero,    zero,   zero,                                               I,      zero,   zero],
                      [zero,    zero,   zero,                                               zero,   I,      zero],
                      [zero,    zero,   zero,                                               zero,   zero,   I]])
'''

# -2 -10 11.5 trekke fra