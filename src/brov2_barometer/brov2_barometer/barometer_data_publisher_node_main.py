import rclpy
from brov2_barometer import barometer_data_publisher_node as node


def main(args=None):
    rclpy.init(args=args)
    
    # Construct the publisher
    barometer_data_publisher = node.BarometerDataPublisher()
    
    # Reading and publishing data at defined rate (0.1 seconds)
    rclpy.spin(barometer_data_publisher)

    # Clean up when script is stopped
    barometer_data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()