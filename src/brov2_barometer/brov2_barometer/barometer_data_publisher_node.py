from rclpy.node import Node

# ms5837 needed in order to utilize the BlueRobotics MS5837 Python Library which must be installed
from brov2_barometer import ms5837
from brov2_interfaces.msg import Barometer


class BarometerDataPublisher(Node):
    # Initializer 
    def __init__(self):
        super().__init__('barometer_data_publisher')
        self.declare_parameter('barometer_topic_name', 'barometer_data')
        self.declare_parameter('barometer_period', 1/10)
        self.declare_parameter('fluid_density', 1029) # Default value for sea water (use 997 for fresh water)

        barometer_topic_name = self.get_parameter('barometer_topic_name').get_parameter_value().string_value
        barometer_period = self.get_parameter('barometer_period').get_parameter_value().double_value
        fluid_density = self.get_parameter('fluid_density').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(Barometer, barometer_topic_name, 10)
        self.timer = self.create_timer(barometer_period, self.barometer_read_and_publish)

        self.sensor = ms5837.MS5837_30BA()
        self.sensor.setFluidDensity(fluid_density)
        if not self.sensor.init():
            self.get_logger().info('Sensor could not be initialized')
            exit(1)
        self.get_logger().info('Barometer initialized')

    def barometer_read_and_publish(self):
        msg = Barometer()
        if self.sensor.read():
                msg.depth                   = self.sensor.depth()                               # Depth in meters using the fluid density (kg/m^3) configured by setFluidDensity()
                msg.pressure_mbar           = self.sensor.pressure()                            # Default is mbar (no arguments)
                msg.pressure_psi            = self.sensor.pressure(ms5837.UNITS_psi)            # Request psi
                msg.temperature_celsius     = self.sensor.temperature()                         # Default is degrees C (no arguments)
                msg.temperature_farenheit   = self.sensor.temperature(ms5837.UNITS_Farenheit)   # Request Farenheit
        else:
                print("Sensor read failed!")
                exit(1)

        self.publisher_.publish(msg)
        