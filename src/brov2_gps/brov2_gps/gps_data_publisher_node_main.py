import rclpy
from brov2_gps import gps_data_publisher_node as node


def main(args=None):
    rclpy.init(args=args)
    
    # Construct the publisher
    gps_data_publisher = node.GPSDataPublisher()
    rclpy.spin(gps_data_publisher)

    # Clean up when script is stopped
    gps_data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()