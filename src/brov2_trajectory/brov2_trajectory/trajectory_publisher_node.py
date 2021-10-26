import csv
import rclpy
from rclpy.node import Node

from bluerov_interfaces.msg import Reference
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion, Twist


class TrajectoryPublisher(Node):

    def __init__(self):
        # Initialization of trajectory publisher
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(Reference, '/CSE/references', 10)
        trajectory_period = 1/10  # seconds
        self.trajectory_timer = self.create_timer(trajectory_period, self.reference_publisher)
        self.trajectory_iterator = 0

        # Getting trajectories from csv-file
        #file = open('horizontal_trajectory.csv')
        file = open('pool_trajectory.csv')
        csvreader = csv.reader(file)
        self.rows = []
        for row in csvreader:
            self.rows.append([float(s) for s in row])
        file.close()

        self.get_logger().info("Trajectory publisher initialized. Start publishing trajectory!")

    def reference_publisher(self):
        position = Vector3()
        position.x = self.rows[self.trajectory_iterator][0]
        position.y = self.rows[self.trajectory_iterator][1]
        position.z = self.rows[self.trajectory_iterator][2]

        orientation = Quaternion()
        orientation.x = self.rows[self.trajectory_iterator][3]
        orientation.y = self.rows[self.trajectory_iterator][4]
        orientation.z = self.rows[self.trajectory_iterator][5]
        orientation.w = self.rows[self.trajectory_iterator][6]

        velocity = Twist()
        velocity.linear.x = self.rows[self.trajectory_iterator][7]
        velocity.linear.y = self.rows[self.trajectory_iterator][8]
        velocity.linear.z = self.rows[self.trajectory_iterator][9]
        velocity.angular.x = self.rows[self.trajectory_iterator][13]
        velocity.angular.y = self.rows[self.trajectory_iterator][14]
        velocity.angular.z = self.rows[self.trajectory_iterator][15]

        acceleration = Twist()
        acceleration.linear.x = self.rows[self.trajectory_iterator][10]
        acceleration.linear.y = self.rows[self.trajectory_iterator][11]
        acceleration.linear.z = self.rows[self.trajectory_iterator][12]
        acceleration.angular.x = self.rows[self.trajectory_iterator][16]
        acceleration.angular.y = self.rows[self.trajectory_iterator][17]
        acceleration.angular.z = self.rows[self.trajectory_iterator][18]

        current_reference = Reference()
        current_reference.header = Header()
        current_reference.header.stamp = self.get_clock().now().to_msg()
        current_reference.pos = position
        current_reference.quat = orientation
        current_reference.velocity = velocity
        current_reference.acceleration = acceleration

        self.publisher_.publish(current_reference)

        if self.trajectory_iterator < len(self.rows)-1:
            self.trajectory_iterator += 1
        else:
            self.get_logger().info("Trajectory completed!")
            self.destroy_node()
            self.get_logger().info("Node destroyed. Start new instance of reference publisher or switch to TeleOP!")
