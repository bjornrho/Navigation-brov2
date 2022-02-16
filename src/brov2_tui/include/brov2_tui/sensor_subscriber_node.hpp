#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "brov2_interfaces/msg/barometer.hpp"
#include "brov2_tui/ncurses_functions.h"
#include "bluerov_interfaces/msg/reference.hpp"

using std::placeholders::_1;

class SensorSubscriber : public rclcpp::Node
{
public:
    SensorSubscriber()
    : Node("sensor_subscriber"),
    nf{}
    {
        // Subscription for sensors, state and trajectory references
        dvl_vel_subscription_ = this->create_subscription<brov2_interfaces::msg::DVL>(
          "dvl/velocity_estimate", 10, std::bind(&SensorSubscriber::dvl_vel_callback, this, _1));
        dvl_pos_subscription_ = this->create_subscription<brov2_interfaces::msg::DVLOdom>(
          "dvl/position_estimate", 10, std::bind(&SensorSubscriber::dvl_pos_callback, this, _1));
        barometer_subscription_ = this->create_subscription<brov2_interfaces::msg::Barometer>(
          "barometer/barometer_data", 10, std::bind(&SensorSubscriber::barometer_callback, this, _1));
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "bno055/imu/data", 10, std::bind(&SensorSubscriber::imu_callback, this, _1));
        trajectory_subscription_ = this->create_subscription<bluerov_interfaces::msg::Reference>(
          "/CSE/references", 10, std::bind(&SensorSubscriber::trajectory_reference_callback, this, _1));
        state_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/CSEI/observer/odom", 10, std::bind(&SensorSubscriber::state_callback, this, _1));

        //timer_ = this->create_wall_timer(100ms, std::bind(&SensorSubscriber::state_window_callback, this));

        latest_reference = bluerov_interfaces::msg::Reference();
        latest_state = nav_msgs::msg::Odometry();
    
        wins = nf.InitWindows();
        nf.SetupTui(wins);
    }

private:
    void dvl_vel_callback(const brov2_interfaces::msg::DVL::ConstSharedPtr msg);
    void dvl_pos_callback(const brov2_interfaces::msg::DVLOdom::ConstSharedPtr msg);
    void barometer_callback(const brov2_interfaces::msg::Barometer::ConstSharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void trajectory_reference_callback(const bluerov_interfaces::msg::Reference::ConstSharedPtr msg);
    void state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);//, bluerov_interfaces::msg::Reference latest_reference);
    //void state_window_callback(bluerov_interfaces::msg::Reference latest_reference, nav_msgs::msg::Odometry latest_state);

    rclcpp::Subscription<brov2_interfaces::msg::DVL>::SharedPtr dvl_vel_subscription_;
    rclcpp::Subscription<brov2_interfaces::msg::DVLOdom>::SharedPtr dvl_pos_subscription_;
    rclcpp::Subscription<brov2_interfaces::msg::Barometer>::SharedPtr barometer_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<bluerov_interfaces::msg::Reference>::SharedPtr trajectory_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscription_;

    //rclcpp::TimerBase::SharedPtr timer_;
    
    bluerov_interfaces::msg::Reference latest_reference;
    nav_msgs::msg::Odometry latest_state;

    NcursesFunctions nf;
    std::vector<WINDOW*> wins;
};
