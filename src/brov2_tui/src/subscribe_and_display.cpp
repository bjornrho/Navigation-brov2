#include <functional>
#include <memory>
#include <ncurses.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "brov2_interfaces/msg/barometer.hpp"

using std::placeholders::_1;

class SensorSubscriber : public rclcpp::Node
{
public:
  SensorSubscriber()
  : Node("sensor_subscriber")
  {
    // dvl_vel_subscription_ = this->create_subscription<std_msgs::msg::String>(
    //   "topic", 10, std::bind(&SensorSubscriber::dvl_vel_callback, this, _1));
    // dvl_pos_subscription_ = this->create_subscription<std_msgs::msg::String>(
    //   "topic", 10, std::bind(&SensorSubscriber::dvl_pos_callback, this, _1));
    barometer_subscription_ = this->create_subscription<brov2_interfaces::msg::Barometer>(
      "barometer_data", 10, std::bind(&SensorSubscriber::barometer_callback, this, _1));
    
    initscr();			/* Start curses mode 		  */
	endwin();			/* End curses mode		  */
  }

private:
  // void dvl_vel_callback(const std_msgs::msg::String::ConstSharedPtr msg) const
  // {
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  // }
// 
  // void dvl_pos_callback(const std_msgs::msg::String::ConstSharedPtr msg) const
  // {
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  // }

  void barometer_callback(const brov2_interfaces::msg::Barometer::ConstSharedPtr msg) const
  {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->depth);
    printw("Hello World !!!");
    
	refresh();
  }

  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dvl_vel_subscription_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dvl_pos_subscription_;
  rclcpp::Subscription<brov2_interfaces::msg::Barometer>::SharedPtr barometer_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSubscriber>());
  rclcpp::shutdown();
  return 0;
}
