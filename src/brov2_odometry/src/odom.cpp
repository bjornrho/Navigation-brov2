#include "odom.hpp"

using std::placeholders::_1;

SimpleOdom::SimpleOdom() : Node{"odom_node"}
{
  // get params
  std::string imu_topic = "INSERT IMU TOPIC";
  std::string dvl_topic = "INSERT DVL TOPIC";
  std::string odom_topic = "INSERT ESTIMATED ODOMETRY TOPIC";
  std::string mocap_topic;
  std::string imu_link;
  std::string dvl_link;

  //if (!nh.getParam("simple_odom/imu_topic", imu_topic))
  //  imu_topic = "/auv/imu";
  //if (!nh.getParam("simple_odom/dvl_topic", dvl_topic))
  //  dvl_topic = "/auv/odom";  
  //if (!nh.getParam("simple_odom/mocap_topic", mocap_topic))
  //  mocap_topic = "/qualisys/Body_1/pose";
  //if (!nh.getParam("simple_odom/odom_topic", odom_topic))
  //  odom_topic = "/odometry/filtered";
  //if (!nh.getParam("simple_odom/imu_link", imu_link))
  //  imu_link = "imu_0";
  //if (!nh.getParam("simple_odom/dvl_link", dvl_link))
  //  dvl_link = "dvl_link";
  //if (!nh.getParam("simple_odom/publish_rate", update_rate))
  //  update_rate = 20;

  // set up IMU and DVL transforms
  tf2_ros::Buffer tf_buffer();
  tf2_ros::TransformListener tf_listener(tf_buffer);
  double timeout = 10; // seconds to wait for transforms to become available
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for IMU and DVL transforms..");
  geometry_msgs::msg::TransformStamped imu_transform = tf_buffer.lookupTransform("base_link", imu_link, rclcpp::Time(0), rclcpp::Duration(timeout)); //rclcpp::Time is forced to wall time and is not recommended for general use.
  geometry_msgs::msg::TransformStamped dvl_transform = tf_buffer.lookupTransform("base_link", dvl_link, rclcpp::Time(0), rclcpp::Duration(timeout));
  tf2::fromMsg(imu_transform.transform.rotation, imu_rotation);
  tf2::fromMsg(dvl_transform.transform.rotation, dvl_rotation);
  tf2::fromMsg(dvl_transform.transform.translation, dvl_translation);

  // subscribers and publishers
  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 1, std::bind(&SimpleOdom::imuCallback, this, _1));
  auto dvl_sub = node->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(dvl_topic, 1, std::bind(&SimpleOdom::dvlCallback, this, _1));
  auto mocap_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(mocap_topic, 1, std::bind(&SimpleOdom::mocapCallback, this, _1));
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1);
  //imu_sub = nh.subscribe(imu_topic, 1, &SimpleOdom::imuCallback, this);
  //dvl_sub = nh.subscribe(dvl_topic, 1, &SimpleOdom::dvlCallback, this);
  //mocap_sub = nh.subscribe(mocap_topic, 1, &SimpleOdom::mocapCallback, this);
  //odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1);

  // wait for first imu, dvl and mocap msg
  //ROS_INFO("Waiting for initial IMU, DVL and MOCAP msgs..");
  //ros::topic::waitForMessage<sensor_msgs::Imu>(imu_topic, nh);
  //ros::topic::waitForMessage<geometry_msgs::TwistWithCovarianceStamped>(dvl_topic, nh);
  //ros::topic::waitForMessage<geometry_msgs::PoseStamped>(mocap_topic, nh);
  
  // There is no waitForMessage-equivalent in ROS2, https://answers.ros.org/question/378693/waitformessage-ros2-equivalent/

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SimpleOdom initialized");
}

void SimpleOdom::spin()
{
  rclcpp::Rate rate(update_rate);

  while (rclcpp::ok())
  {
    auto node = std::make_shared<SimpleOdom>();
    // execute waiting callbacks
    rclcpp::spin_some(node);

    // create odom msg
    nav_msgs::msg::Odometry odometry_msg;  

    odometry_msg.pose.pose.position = tf2::toMsg(position);     // Might need to convert this to stamped versions
    odometry_msg.pose.pose.orientation = tf2::toMsg(orientation);

    odometry_msg.twist.twist.angular.x = angular_vel[0];
    odometry_msg.twist.twist.angular.y = angular_vel[1];
    odometry_msg.twist.twist.angular.z = angular_vel[2];

    odometry_msg.twist.twist.linear.x = linear_vel[0];
    odometry_msg.twist.twist.linear.y = linear_vel[1];
    odometry_msg.twist.twist.linear.z = linear_vel[2];

    // publish
    odom_pub->publish(odometry_msg);
    loop_rate.sleep();
  }
}

void SimpleOdom::imuCallback(const sensor_msgs::msg::Imu& imu_msg)
{
  tf2::Vector3 angular_vel_imu;
  tf2::fromMsg(imu_msg.angular_velocity, angular_vel_imu);
  angular_vel = tf2::quatRotate(imu_rotation, angular_vel_imu);
}

void SimpleOdom::dvlCallback(const geometry_msgs::msg::TwistWithCovarianceStamped& twist_msg)
{
  tf2::Vector3 linear_vel_dvl, linear_vel_uncorrected;
  tf2::fromMsg(twist_msg.twist.twist.linear, linear_vel_dvl);
  linear_vel_uncorrected = tf2::quatRotate(dvl_rotation, linear_vel_dvl);

  // compensate for translation of DVL
  linear_vel = linear_vel_uncorrected + angular_vel.cross(dvl_translation);  // from fossen2021 eq 14.3
}

void SimpleOdom::mocapCallback(const geometry_msgs::msg::PoseStamped& msg)
{
  tf2::fromMsg(msg.pose.position, position);
  tf2::fromMsg(msg.pose.orientation, orientation);
}