#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "brov2_interfaces/msg/dvl.hpp"
#include "brov2_interfaces/msg/dvl_beam.hpp"
#include "brov2_interfaces/msg/dvl_odom.hpp"


using namespace std::chrono_literals;

class DVLPublisher : public rclcpp::Node
{
  public:
    DVLPublisher()
    : Node("dvl_publisher")
    { 
      this->declare_parameter<std::string>("IP", "192.168.2.52");
      this->declare_parameter<std::string>("port", "16171");   
      this->declare_parameter<std::string>("dvl_pos_topic_name", "position_estimate");
      this->declare_parameter<std::string>("dvl_vel_topic_name", "velocity_estimate");

      this->get_parameter("IP", IP_);
      this->get_parameter("port", port_);
      this->get_parameter("dvl_pos_topic_name", dvl_pos_topic_name_);
      this->get_parameter("dvl_vel_topic_name", dvl_vel_topic_name_);
      velocity_publisher_ = this->create_publisher<brov2_interfaces::msg::DVL>(dvl_vel_topic_name_, 4);
      odometry_publisher_ = this->create_publisher<brov2_interfaces::msg::DVLOdom>(dvl_pos_topic_name_, 2);
      connectToDVL();
      timer_ = this->create_wall_timer(
      30ms, std::bind(&DVLPublisher::publisher, this));
    }

    void connectToDVL();  

  private:
    std::string getData();
    void publisher();

    int sock;
    std::string IP_;
    std::string port_;
    std::string dvl_pos_topic_name_;
    std::string dvl_vel_topic_name_;

    brov2_interfaces::msg::DVL theDVL = brov2_interfaces::msg::DVL();
    brov2_interfaces::msg::DVLOdom DVLPosEstimate = brov2_interfaces::msg::DVLOdom();
    brov2_interfaces::msg::DVLBeam beam0 = brov2_interfaces::msg::DVLBeam();
    brov2_interfaces::msg::DVLBeam beam1 = brov2_interfaces::msg::DVLBeam();
    brov2_interfaces::msg::DVLBeam beam2 = brov2_interfaces::msg::DVLBeam();
    brov2_interfaces::msg::DVLBeam beam3 = brov2_interfaces::msg::DVLBeam();    

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<brov2_interfaces::msg::DVL>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<brov2_interfaces::msg::DVLOdom>::SharedPtr odometry_publisher_;
};