#include "brov2_tui/sensor_subscriber_node.hpp"

using std::placeholders::_1;




void SensorSubscriber::dvl_vel_callback(const brov2_interfaces::msg::DVL::ConstSharedPtr msg)
{
    nf.UpdateDVLWindow(wins[1], msg);
}

void SensorSubscriber::dvl_pos_callback(const brov2_interfaces::msg::DVLOdom::ConstSharedPtr msg)
{
  nf.UpdateOdomWindow(wins[2], msg);
}

void SensorSubscriber::barometer_callback(const brov2_interfaces::msg::Barometer::ConstSharedPtr msg)
{
    nf.UpdateBarometerWindow(wins[3], msg);
}

void SensorSubscriber::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    nf.UpdateIMUWindow(wins[4], msg);
}

void SensorSubscriber::trajectory_reference_callback(const bluerov_interfaces::msg::Reference::ConstSharedPtr msg)
{
    latest_reference = *msg;
}

void SensorSubscriber::state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)//, bluerov_interfaces::msg::Reference latest_reference)
{
    latest_state = *msg;
}

//void SensorSubscriber::state_window_callback(bluerov_interfaces::msg::Reference latest_reference, nav_msgs::msg::Odometry latest_state)
//{
//    
//
//    nf.UpdateStateWindow(wins[5], msg);
//
//}