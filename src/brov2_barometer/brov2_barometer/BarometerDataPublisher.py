import rclpy
from rclpy.node import Node


from brov2_barometer import ms5837
from brov2_interfaces.msg import Barometer



class BarometerDataPublisher(Node):

    def __init__(self):
        super().__init__('BarometerDataPublisher')
        self.publisher_ = self.create_publisher(Barometer, 'barometer_data', 10)
        read_period = 0.1  # seconds
        self.timer = self.create_timer(read_period, self.barometer_read_and_publish)

        self.sensor = ms5837.MS5837_30BA()
        if not self.sensor.init():
            print("Sensor could not be initialized")
            exit(1)

    def barometer_read_and_publish(self):
        msg = Barometer()

        if self.sensor.read():
                msg.pressure_mbar           = self.sensor.pressure()                           # Default is mbar (no arguments)
                msg.pressure_psi            = self.sensor.pressure(ms5837.UNITS_psi)           # Request psi
                msg.temperature_celsius     = self.sensor.temperature()                        # Default is degrees C (no arguments)
                msg.temperature_farenheit   = self.sensor.temperature(ms5837.UNITS_Farenheit)  # Request Farenheit
        else:
                print("Sensor read failed!")
                exit(1)

        self.publisher_.publish(msg)
        self.get_logger().info('P: %0.1f mbar  %0.3f psi\tT: %0.2f C  %0.2f F' % (msg.pressure_mbar, msg.pressure_psi, msg.temperature_celsius, msg))


def main(args=None):
    rclpy.init(args=args)
    
    barometer_data_publisher = BarometerDataPublisher()

    rclpy.spin(barometer_data_publisher)

    barometer_data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
