import rclpy
from brov2_sonar_processing import sonar_processing_node as node
import matplotlib.pyplot as plt
    

def main(args=None):
    rclpy.init(args=args)
    plt.show(block=True)
    sonar_processor = node.SonarProcessingNode()

    rclpy.spin(sonar_processor)
    
    sonar_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
