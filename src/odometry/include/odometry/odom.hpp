#ifndef ODOM_HPP
#define ODOM_HPP

//#include <ros/ros.h>
//#include <ros/console.h>
#include "rclcpp/rclcpp.hpp"


#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>
// #include <eigen_conversions/eigen_msg.h>

/**
 * @brief Class that combines measurements from IMU and DVL into a simple
 * odometry publisher. Positions in x and y are estimated by euler integration.
 *
 */
class SimpleOdom : public rclcpp::Node
{
public:
  SimpleOdom();
  void spin();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  void imuCallback(const sensor_msgs::msg::Imu& imu_msg);
  void dvlCallback(const geometry_msgs::msg::TwistWithCovarianceStamped& twist_msg);
  void mocapCallback(const geometry_msgs::msg::PoseStamped& msg);
  Eigen::Vector3d position;
  tf2::Quaternion orientation;
  tf2::Vector3 linear_vel;
  tf2::Vector3 angular_vel;
  tf2::Quaternion imu_rotation;
  tf2::Quaternion dvl_rotation;
  tf2::Vector3 dvl_translation;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  double update_rate;
};

#endif