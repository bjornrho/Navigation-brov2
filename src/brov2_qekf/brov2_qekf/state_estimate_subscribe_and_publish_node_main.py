import rclpy
from brov2_qekf import state_estimate_subscribe_and_publish_node as state_node


def main(args=None):
    rclpy.init(args=args)

    state_estimate_sub_pub = state_node.StateEstimateSubPub()

    rclpy.spin(state_estimate_sub_pub)

    state_estimate_sub_pub.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()