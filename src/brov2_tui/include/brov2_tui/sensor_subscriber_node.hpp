#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "brov2_interfaces/msg/barometer.hpp"

#include "brov2_tui/ncurses_functions.h"

using std::placeholders::_1;

class SensorSubscriber : public rclcpp::Node
{
public:
    SensorSubscriber()
    : Node("sensor_subscriber"),
    nf{}
    {
        dvl_vel_subscription_ = this->create_subscription<brov2_interfaces::msg::DVL>(
          "dvl/velocity_estimate", 10, std::bind(&SensorSubscriber::dvl_vel_callback, this, _1));
        dvl_pos_subscription_ = this->create_subscription<brov2_interfaces::msg::DVLOdom>(
          "dvl/position_estimate", 10, std::bind(&SensorSubscriber::dvl_pos_callback, this, _1));
        barometer_subscription_ = this->create_subscription<brov2_interfaces::msg::Barometer>(
          "barometer/barometer_data", 10, std::bind(&SensorSubscriber::barometer_callback, this, _1));
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "bno055/imu/data", 10, std::bind(&SensorSubscriber::imu_callback, this, _1));
    
        wins = nf.InitWindows();
        nf.SetupTui(wins);
    }

private:
    void dvl_vel_callback(const brov2_interfaces::msg::DVL::ConstSharedPtr msg);
    void dvl_pos_callback(const brov2_interfaces::msg::DVLOdom::ConstSharedPtr msg);
    void barometer_callback(const brov2_interfaces::msg::Barometer::ConstSharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

    rclcpp::Subscription<brov2_interfaces::msg::DVL>::SharedPtr dvl_vel_subscription_;
    rclcpp::Subscription<brov2_interfaces::msg::DVLOdom>::SharedPtr dvl_pos_subscription_;
    rclcpp::Subscription<brov2_interfaces::msg::Barometer>::SharedPtr barometer_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    NcursesFunctions nf;
    std::vector<WINDOW*> wins;
};
