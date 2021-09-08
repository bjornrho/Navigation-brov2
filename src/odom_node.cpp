#include "odometry/odom.hpp"

int main(int argc, char** argv)
{
  //const bool DEBUG_MODE = true;  // debug logs are printed to console when true

  rclcpp::init(argc, argv);

  //if (DEBUG_MODE)
  //{
  //  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  //  ros::console::notifyLoggerLevelsChanged();
  //}

  // ros::Rate rate(50);
  SimpleOdom simple_odom = {};
  simple_odom.spin();
}