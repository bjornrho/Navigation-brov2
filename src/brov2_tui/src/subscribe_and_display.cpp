#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
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
        dvl_vel_subscription_ = this->create_subscription<brov2_interfaces::msg::DVL>(
          "dvl_velocity_estimate", 10, std::bind(&SensorSubscriber::dvl_vel_callback, this, _1));
        dvl_pos_subscription_ = this->create_subscription<brov2_interfaces::msg::DVLOdom>(
          "dvl_position_estimate", 10, std::bind(&SensorSubscriber::dvl_pos_callback, this, _1));
        barometer_subscription_ = this->create_subscription<brov2_interfaces::msg::Barometer>(
          "barometer_data", 10, std::bind(&SensorSubscriber::barometer_callback, this, _1));
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "imu/data", 10, std::bind(&SensorSubscriber::imu_callback, this, _1));
    
        wins = nf.InitWindows();
        nf.SetupTui(wins);
    }

private:
    void dvl_vel_callback(const brov2_interfaces::msg::DVL::ConstSharedPtr msg)
    {
        nf.UpdateDVLWindow(wins[1], msg);
    }

    void dvl_pos_callback(const brov2_interfaces::msg::DVLOdom::ConstSharedPtr msg)
    {
      nf.UpdateOdomWindow(wins[2], msg);
    }

    void barometer_callback(const brov2_interfaces::msg::Barometer::ConstSharedPtr msg)
    {
        nf.UpdateBarometerWindow(wins[3], msg);
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        nf.UpdateIMUWindow(wins[4], msg);
    }

    rclcpp::Subscription<brov2_interfaces::msg::DVL>::SharedPtr dvl_vel_subscription_;
    rclcpp::Subscription<brov2_interfaces::msg::DVLOdom>::SharedPtr dvl_pos_subscription_;
    rclcpp::Subscription<brov2_interfaces::msg::Barometer>::SharedPtr barometer_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
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
