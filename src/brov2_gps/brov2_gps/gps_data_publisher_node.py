import time
import board
import busio
import adafruit_gps
import serial

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix



class GPSDataPublisher(Node):
    # Initializer 
    def __init__(self):
        super().__init__('gps_data_publisher')
        self.declare_parameters(namespace='',
                                parameters=[('gps_topic_name', 'gps_data'),
                                            ('gps_period', 1),
                                            ('serial_connection', '/dev/ttyUSB1')])

        gps_topic_name, gps_period, serial_connection = self.get_parameters(['gps_topic_name',
                                                                             'gps_period',
                                                                             'serial_connection'])


        self.publisher_ = self.create_publisher(NavSatFix, gps_topic_name.value, 10)
        self.timer = self.create_timer(gps_period.value, self.gps_read_and_publish)

        ### GPS related initialization
        # Create a serial connection and GPS module instance.
        uart = serial.Serial(serial_connection.value, baudrate=9600, timeout=10)
        self.gps = adafruit_gps.GPS(uart, debug=False)
        # Turn on the basic GGA and RMC info (what you typically want)
        self.gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        # Set update rate to once a second (1hz) which is what you typically want.
        self.gps.send_command(b"PMTK220,1000")
        # Bool to keep track of lost and found fix
        self.gps_previous_fix = False

        self.get_logger().info("GPS data pubisher node initialized. Waiting for fix.")


    def gps_read_and_publish(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()

        self.gps.update()

        # Lost & found of fix. You gotta know!
        if self.gps.has_fix and not self.gps_previous_fix:
            self.get_logger().info("GPS fix acquired!")
        elif self.gps_previous_fix and not self.gps.has_fix:
            self.get_logger().info("GPS fix lost!")
        self.gps_previous_fix = self.gps.has_fix

        # Generating message
        if not self.gps.has_fix:
            msg.status.status = -1
        else:
            msg.status.status   = 0
            msg.latitude        = self.gps.latitude
            msg.longitude       = self.gps.longitude
            msg.altitude        = self.gps.altitude_m
            
        # Publishing message from GPS
        self.publisher_.publish(msg)
