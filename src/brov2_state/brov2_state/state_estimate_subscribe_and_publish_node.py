from numpy.core.numeric import roll
from rosidl_parser.definition import Include

import math
import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from brov2_interfaces.msg import DVL, DVLOdom, Barometer


class StateEstimateSubPub(Node):

    def __init__(self):
        super().__init__('state_estimate_sub_pub')
        # Position offset parameters
        self.declare_parameter('position_offset_x', value=0.0)
        self.declare_parameter('position_offset_y', value=0.0)

        # Orientation offset parameters
        self.declare_parameter('orientation_offset_w', value=1.0)
        self.declare_parameter('orientation_offset_x', value=0.0)
        self.declare_parameter('orientation_offset_y', value=0.0)
        self.declare_parameter('orientation_offset_z', value=0.0)

        # Services for setting offset
        self.srv = self.create_service(Trigger, 'brov2_state/set_pos_offset',self.set_position_offset)
        self.srv = self.create_service(Trigger, 'brov2_state/set_orient_offset',self.set_orientation_offset)   

        # Initializing subscribers for sensors
        self.dvl_subscription = self.create_subscription(
            DVL,
            'dvl/velocity_estimate',
            self.dvl_vel_sub,
            10)
        
        self.dvl_odom_subscription = self.create_subscription(
            DVLOdom,
            'dvl/position_estimate',
            self.dvl_odom_sub,
            10)

        self.imu_subscription = self.create_subscription(
            Imu,
            'bno055/imu/data',
            self.imu_sub,
            10)

        self.barometer_subscription = self.create_subscription(
            Barometer,
            'barometer/barometer_data',
            self.barometer_sub,
            10)

        # Initialization of state estimate publisher
        self.state_estimate_publisher_ = self.create_publisher(Odometry, '/CSEI/observer/odom', 10)
        state_estimate_period = 1/10  # seconds
        self.state_estimate_timer = self.create_timer(state_estimate_period, self.state_estimate_publisher)
        
        # Initializing current message variables
        self.current_vel = DVL()
        self.current_odom = DVLOdom()
        self.current_imu = Imu()
        self.current_barometer = Barometer()
        self.current_state_estimate = Odometry()

        self.current_corrected_orientation = np.zeros(4)

    def quaternion_multiply(self, quaternion1, quaternion0):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                          x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                         -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                          x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)
 
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

    def dvl_vel_sub(self, msg):
        self.current_vel = msg

    def dvl_odom_sub(self, msg):
        self.current_odom = msg

    def imu_sub(self, msg):
        self.current_imu = msg

    def barometer_sub(self, msg):
        self.current_barometer = msg

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
        conjugated_orientation = self.current_corrected_orientation
        for i in range(1,3):
            conjugated_orientation[i] = -1 * self.current_corrected_orientation[i]

        w_offset, x_offset, y_offset, z_offset = self.quaternion_multiply(np.array([1, 0, 0, 0]), conjugated_orientation)
        
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
        orientation_offset_quat = np.array([orientation_offset_w,
                                            orientation_offset_x,
                                            orientation_offset_y,
                                            orientation_offset_z])
        
        # Position
        self.current_state_estimate.pose.pose.position.x = self.current_odom.x - position_offset_x
        self.current_state_estimate.pose.pose.position.y = self.current_odom.y - position_offset_y
        self.current_state_estimate.pose.pose.position.z = self.current_barometer.depth

        # IMU axis correction - from ENU to NED (IMU orientation initialized in ros2_bno055_sensor)
        quaternion_imu_raw = np.array([self.current_imu.orientation.w,
                                   self.current_imu.orientation.x,
                                   self.current_imu.orientation.y,
                                   self.current_imu.orientation.z])

        quaternion_rot_correction = np.array([0, np.sqrt(2)/2, np.sqrt(2)/2, 0])
        #np.array([0, 0, -np.sqrt(2)/2, -np.sqrt(2)/2])
        
        w_oriented, x_oriented, y_oriented, z_oriented = self.quaternion_multiply(quaternion_rot_correction, quaternion_imu_raw)
        self.current_corrected_orientation = [w_oriented, x_oriented, y_oriented, z_oriented]

        # Orientation
        w, x, y, z = self.quaternion_multiply(self.current_corrected_orientation, orientation_offset_quat)
        self.current_state_estimate.pose.pose.orientation.w = w
        self.current_state_estimate.pose.pose.orientation.x = x
        self.current_state_estimate.pose.pose.orientation.y = y
        self.current_state_estimate.pose.pose.orientation.z = z

        # Velocity linear
        self.current_state_estimate.twist.twist.linear.x = self.current_vel.velocity.x
        self.current_state_estimate.twist.twist.linear.y = self.current_vel.velocity.y
        self.current_state_estimate.twist.twist.linear.z = self.current_vel.velocity.z

        # Velocity angular
        self.current_state_estimate.twist.twist.angular.x = self.current_imu.angular_velocity.x
        self.current_state_estimate.twist.twist.angular.y = self.current_imu.angular_velocity.y
        self.current_state_estimate.twist.twist.angular.z = self.current_imu.angular_velocity.z

        self.state_estimate_publisher_.publish(self.current_state_estimate)

        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(w,x,y,z)

        print("Converted quaternion to euler angles: ")
        print("Roll around x:  ", roll_x)
        print("Pitch around y: ", pitch_y)
        print("Yaw around z:   ", yaw_z)
        print("--------------------------------------")
