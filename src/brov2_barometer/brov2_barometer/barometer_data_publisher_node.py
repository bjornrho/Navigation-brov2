from rclpy.node import Node

# ms5837 needed in order to utilize the BlueRobotics MS5837 Python Library which must be installed
from brov2_barometer import ms5837
from brov2_interfaces.msg import Barometer



class BarometerDataPublisher(Node):
    # Initializer 
    def __init__(self):
        super().__init__('BarometerDataPublisher')
        self.publisher_ = self.create_publisher(Barometer, 'barometer_data', 10)
        read_period = 0.1  # seconds
        self.timer = self.create_timer(read_period, self.barometer_read_and_publish)

        self.sensor = ms5837.MS5837_30BA()
        # self.sensor.setFluidDensity() # Configuring fluid density for fresh or saltwater. Defaulting to fresh water
        if not self.sensor.init():
            print("Sensor could not be initialized")
            exit(1)

    def barometer_read_and_publish(self):
        # Custom barometer message to publish. Can be found in the brov2_interfaces.
        msg = Barometer()

        # Reading barometer and loading data into custom message
        if self.sensor.read():
                msg.depth                   = self.sensor.depth()                               # Depth in meters using the fluid density (kg/m^3) configured by setFluidDensity()
                msg.pressure_mbar           = self.sensor.pressure()                            # Default is mbar (no arguments)
                msg.pressure_psi            = self.sensor.pressure(ms5837.UNITS_psi)            # Request psi
                msg.temperature_celsius     = self.sensor.temperature()                         # Default is degrees C (no arguments)
                msg.temperature_farenheit   = self.sensor.temperature(ms5837.UNITS_Farenheit)   # Request Farenheit
        else:
                print("Sensor read failed!")
                exit(1)

        # Publishing message and logging data sent over the topic /barometer_data
        self.publisher_.publish(msg)
        self.get_logger().info('Depth: %0.2f m\tP: %0.1f mbar  %0.3f psi\tT: %0.2f C  %0.2f F' % (msg.depth, 
                                                                                                  msg.pressure_mbar, 
                                                                                                  msg.pressure_psi, 
                                                                                                  msg.temperature_celsius, 
                                                                                                  msg.temperature_farenheit))
