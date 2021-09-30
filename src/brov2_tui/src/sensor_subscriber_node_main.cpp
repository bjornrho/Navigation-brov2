#include "rclcpp/rclcpp.hpp"
#include "brov2_tui/sensor_subscriber_node.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSubscriber>());
  rclcpp::shutdown();
  return 0;
}