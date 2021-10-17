from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from brov2_interfaces.msg import DVL, DVLOdom


class StateEstimateSubPub(Node):

    def __init__(self):
        super().__init__('state_estimate_sub_pub')

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

        # Initialization of state estimate publisher
        self.state_estimate_publisher_ = self.create_publisher(Odometry, 'state_estimate_topic', 10)
        state_estimate_period = 1/10  # seconds
        self.state_estimate_timer = self.create_timer(state_estimate_period, self.state_estimate_publisher)
        
        # Initializing current message variables
        self.current_vel = DVL()
        self.current_odom = DVLOdom()
        self.current_imu = Imu()
        self.current_state_estimate = Odometry()

    def dvl_vel_sub(self, msg):
        self.current_vel = msg

    def dvl_odom_sub(self, msg):
        self.current_odom = msg

    def imu_sub(self, msg):
        self.current_imu = msg

    def state_estimate_publisher(self):
        self.current_state_estimate.header.stamp = self.get_clock().now().to_msg()
        # Position
        self.current_state_estimate.pose.pose.position.x = self.current_odom.x
        self.current_state_estimate.pose.pose.position.y = self.current_odom.y
        self.current_state_estimate.pose.pose.position.z = self.current_odom.z
        # Orientation - MAKE SURE THE ORIENTATION IS CORRECT
        self.current_state_estimate.pose.pose.orientation.x = self.current_imu.orientation.x
        self.current_state_estimate.pose.pose.orientation.y = self.current_imu.orientation.y
        self.current_state_estimate.pose.pose.orientation.z = self.current_imu.orientation.z
        self.current_state_estimate.pose.pose.orientation.w = self.current_imu.orientation.w
        # Velocity linear
        self.current_state_estimate.twist.twist.linear.x = self.current_vel.velocity.x
        self.current_state_estimate.twist.twist.linear.y = self.current_vel.velocity.y
        self.current_state_estimate.twist.twist.linear.z = self.current_vel.velocity.z
        # Velocity angular
        self.current_state_estimate.twist.twist.angular.x = self.current_imu.angular_velocity.x
        self.current_state_estimate.twist.twist.angular.y = self.current_imu.angular_velocity.y
        self.current_state_estimate.twist.twist.angular.z = self.current_imu.angular_velocity.z

        self.state_estimate_publisher_.publish(self.current_state_estimate)

