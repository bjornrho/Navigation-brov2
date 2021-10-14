#include "brov2_sonar/sonar_publisher_node.hpp"


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SonarPublisher>());
    rclcpp::shutdown();
	return 0;
}