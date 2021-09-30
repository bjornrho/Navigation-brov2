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
      velocity_publisher_ = this->create_publisher<brov2_interfaces::msg::DVL>("velocity_estimate", 10);
      odometry_publisher_ = this->create_publisher<brov2_interfaces::msg::DVLOdom>("position_estimate", 10);
      connectToDVL();
      timer_ = this->create_wall_timer(
      100ms, std::bind(&DVLPublisher::publisher, this));
    }

    void connectToDVL();  

  private:
    std::string getData();
    void publisher();

    int sock;

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