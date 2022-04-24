import csv
import rclpy
from rclpy.node import Node
import numpy as np

from bluerov_interfaces.msg import Reference
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion, Twist

from sensor_msgs.msg import Imu
from brov2_interfaces.msg import DVL, Barometer


class TrajectoryPublisher(Node):

    def __init__(self):
        # Initialization of trajectory publisher
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(Reference, '/CSE/references', 10)
        trajectory_period = 1/10  # seconds
        self.trajectory_timer = self.create_timer(trajectory_period, self.reference_publisher)
        self.trajectory_iterator = 0

        # Initialization of simulated measurements
        simulate = True
        if True and simulate:
            self.current_imu = Imu()
            self.publisher_imu = self.create_publisher(Imu, '/bno055/imu', 10)
            imu_period = 1/10
            self.imu_timer = self.create_timer(imu_period, self.simulate_imu)
        if True and simulate:
            self.current_vel = DVL()
            self.publisher_dvl = self.create_publisher(DVL, 'velocity_estimate', 10)
            dvl_period = 1/6
            self.dvl_timer = self.create_timer(dvl_period, self.simulate_dvl)
        if False and simulate:
            self.current_barometer = Barometer()
            self.publisher_barometer = self.create_publisher(Barometer, 'barometer/barometer_data', 10)
            barometer_period = 1/5
            self.barometer_timer = self.create_timer(barometer_period, self.simulate_barometer)


        # Getting trajectories from csv-file
        file = open('trajectories/horizontal_trajectory.csv')
        #file = open('trajectories/pool_trajectory.csv')
        csvreader = csv.reader(file)
        self.rows = []
        for row in csvreader:
            self.rows.append([float(s) for s in row])
        file.close()

        self.get_logger().info("Trajectory publisher initialized. Start publishing trajectory!")

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
        else:
            self.get_logger().info("Trajectory completed!")
            self.destroy_node()
            self.get_logger().info("Node destroyed. Start new instance of reference publisher or switch to TeleOP!")


    def simulate_imu(self):
        self.current_imu.header.stamp = self.get_clock().now().to_msg()
        self.current_imu.linear_acceleration = Vector3(x=self.rows[self.trajectory_iterator][10],
                                                       y=self.rows[self.trajectory_iterator][11],
                                                       z=self.rows[self.trajectory_iterator][12]+9.81)

        self.current_imu.angular_velocity = Vector3(x=self.rows[self.trajectory_iterator][13],
                                                    y=self.rows[self.trajectory_iterator][14],
                                                    z=self.rows[self.trajectory_iterator][15])

        self.current_imu.orientation = Quaternion(w=self.rows[self.trajectory_iterator][3],
                                                  x=self.rows[self.trajectory_iterator][4],
                                                  y=self.rows[self.trajectory_iterator][5],
                                                  z=self.rows[self.trajectory_iterator][6])

        self.publisher_imu.publish(self.current_imu)
        

    def simulate_dvl(self):
        self.current_vel.velocity = Vector3(x=self.rows[self.trajectory_iterator][7],
                                            y=self.rows[self.trajectory_iterator][8],
                                            z=self.rows[self.trajectory_iterator][9])
        
        self.current_vel.covariance = np.eye(3).ravel()
        self.current_vel.velocity_valid = True
        
        self.publisher_dvl.publish(self.current_vel)

    def simulate_barometer(self):
        self.current_barometer.depth = self.rows[self.trajectory_iterator][2]
        self.publisher_barometer.publish(self.current_barometer)
