import rclpy
from brov2_trajectory import trajectory_publisher_node as node




def main(args=None):
    rclpy.init(args=args)

    trajectory_publisher = node.TrajectoryPublisher()

    rclpy.spin(trajectory_publisher)

    trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()