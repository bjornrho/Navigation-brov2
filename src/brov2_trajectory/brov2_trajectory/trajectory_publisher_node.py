import csv
import rclpy
from rclpy.node import Node
import numpy as np

from bluerov_interfaces.msg import Reference
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion, Twist

from sensor_msgs.msg import Imu
from brov2_interfaces.msg import DVL, Barometer

from numpy.linalg import norm
import progressbar

import sys
sys.path.append('utility_functions')
import utility_functions


class TrajectoryPublisher(Node):

    def __init__(self):
        # Initialization of trajectory publisher
        super().__init__('trajectory_publisher')
        self.declare_parameters(namespace='',
                                parameters=[('trajectory_topic_name', '/CSE/references'),
                                            ('trajectory_file_name', 'MC_circle.csv'),
                                            ('trajectory_period', 1/100),
                                            ('update_printout', True)])

        trajectory_topic_name, trajectory_file_name, self.trajectory_period = self.get_parameters(['trajectory_topic_name',
                                                                                              'trajectory_file_name',
                                                                                              'trajectory_period'])
        self.update_printout = self.get_parameter('update_printout').get_parameter_value().bool_value

        self.publisher_ = self.create_publisher(Reference, trajectory_topic_name.value, 10)
        self.trajectory_timer = self.create_timer(self.trajectory_period.value, self.reference_publisher)
        self.trajectory_iterator = 0

        # Getting trajectories from csv-file
        file = open('trajectories/'+trajectory_file_name.value)
        csvreader = csv.reader(file)
        self.rows = []
        for row in csvreader:
            self.rows.append([float(s) for s in row])
        file.close()

        # Printout for progress
        duration_minutes = (((len(self.rows)) / 100) / 60)
        duration_seconds = round(((len(self.rows) / 100) % 60))
        self.pbar = progressbar.ProgressBar(max_value = round(len(self.rows)), redirect_stdout = True)
        
        self.get_logger().info("Trajectory publisher initialized.")
        self.get_logger().info("Executing %s with duration of approximately: %i minutes and %i seconds \n" % 
                (trajectory_file_name.value, duration_minutes, duration_seconds))


    def reference_publisher(self):
        position = Vector3(x=self.rows[self.trajectory_iterator][0],
                           y=self.rows[self.trajectory_iterator][1],
                           z=self.rows[self.trajectory_iterator][2])

        orientation = Quaternion(w=self.rows[self.trajectory_iterator][3],
                                 x=self.rows[self.trajectory_iterator][4],
                                 y=self.rows[self.trajectory_iterator][5],
                                 z=self.rows[self.trajectory_iterator][6])


        velocity = Twist()
        velocity.linear = Vector3(x=self.rows[self.trajectory_iterator][7],
                                  y=self.rows[self.trajectory_iterator][8],
                                  z=self.rows[self.trajectory_iterator][9])
        velocity.angular = Vector3(x=self.rows[self.trajectory_iterator][13],
                                   y=self.rows[self.trajectory_iterator][14],
                                   z=self.rows[self.trajectory_iterator][15])

        acceleration = Twist()
        acceleration.linear = Vector3(x=self.rows[self.trajectory_iterator][10],
                                      y=self.rows[self.trajectory_iterator][11],
                                      z=self.rows[self.trajectory_iterator][12])
        acceleration.angular = Vector3(x=self.rows[self.trajectory_iterator][16],
                                       y=self.rows[self.trajectory_iterator][17],
                                       z=self.rows[self.trajectory_iterator][18])

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
            if self.trajectory_iterator % 10 == 0:
                if self.update_printout and self.trajectory_iterator > (1/self.trajectory_period.value)*3:
                    yaw = utility_functions.yaw_from_quaternion(np.array([[orientation.w],[orientation.x],
                                                                          [orientation.y],[orientation.z]]))
                    print("Current position and yaw: [%f,%f,%f] & %f" %(position.x, position.y, position.z, yaw*180/np.pi))
                self.pbar.update(self.trajectory_iterator)
        else:
            self.pbar.finish()
            self.get_logger().info("Trajectory completed!")
            self.destroy_node()
            self.get_logger().info("Node destroyed. Start new instance of trajectory publisher or switch to TeleOP!")
