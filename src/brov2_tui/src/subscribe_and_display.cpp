#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "brov2_interfaces/msg/barometer.hpp"

#include "brov2_tui/ncursesfun.h"

using std::placeholders::_1;

class SensorSubscriber : public rclcpp::Node
{
public:
  SensorSubscriber()
  : Node("sensor_subscriber"),
  nf{}
  {
    // dvl_vel_subscription_ = this->create_subscription<std_msgs::msg::String>(
    //   "topic", 10, std::bind(&SensorSubscriber::dvl_vel_callback, this, _1));
    // dvl_pos_subscription_ = this->create_subscription<std_msgs::msg::String>(
    //   "topic", 10, std::bind(&SensorSubscriber::dvl_pos_callback, this, _1));
    barometer_subscription_ = this->create_subscription<brov2_interfaces::msg::Barometer>(
      "barometer_data", 10, std::bind(&SensorSubscriber::barometer_callback, this, _1));

    wins = nf.InitWindows();
    nf.SetupTui(wins);
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

  void barometer_callback(const brov2_interfaces::msg::Barometer::ConstSharedPtr msg)
  {
    nf.UpdateBarometerWindow(wins[3], msg);
  }

  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dvl_vel_subscription_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dvl_pos_subscription_;
  rclcpp::Subscription<brov2_interfaces::msg::Barometer>::SharedPtr barometer_subscription_;
  NcursesFunctions nf;
  std::vector<WINDOW*> wins;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSubscriber>());
  rclcpp::shutdown();

  // NcursesFunctions nf;
  // std::vector<WINDOW*> my_wins = nf.InitWindows();
  // 
  // nf.SetupTui(my_wins);



  getch();
  endwin();


  return 0;
}
