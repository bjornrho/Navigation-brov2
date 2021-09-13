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

int sock;
std::string oldJson = "";

class DVLPublisher : public rclcpp::Node
{
  public:
    DVLPublisher()
    : Node("dvl_publisher"), count_(0)
    {
      velocity_publisher_ = this->create_publisher<brov2_interfaces::msg::DVL>("dvl_velocity_estimate", 10);
      odometry_publisher_ = this->create_publisher<brov2_interfaces::msg::DVLOdom>("dvl_position_estimate", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&DVLPublisher::publisher, this));
    }

    void connectToDVL(){
        struct addrinfo hints, *res;

        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        getaddrinfo("10.42.1.180", "16171", &hints, &res); // Move IP and PORT values to parameter server

        try{
            sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
            if (connect(sock, res->ai_addr, res->ai_addrlen) < 0){
                RCLCPP_INFO(this->get_logger(), "Connection failed");
                throw connect(sock, res->ai_addr, res->ai_addrlen);
            }else{
                RCLCPP_INFO(this->get_logger(), "Connection succeeded");
            }
            
        }
        catch (int except) {
            RCLCPP_INFO(this->get_logger(), "Trying to connect again");
            connectToDVL();
        }
    }
    
    

  private:
    std::string getData(){
        char buf[1];
        int rec;

        std::string raw_data = "";

        while (raw_data.back() != '\n'){
            rec = recv(sock,buf,1,0);
            if (rec == 0){
                RCLCPP_INFO(this->get_logger(), "Socket closed by the DVL, reopening");
                this->connectToDVL();
                // Add exception handling
            }
            raw_data = raw_data + buf;
        }       

        return raw_data;
    }

    void publisher(){
        std::string raw_data = this->getData();

        nlohmann::json json_data = nlohmann::json::parse(raw_data);

        if (json_data["type"] == "velocity"){
            theDVL.time             = json_data["time"];
	        theDVL.velocity.x       = json_data["vx"];
            theDVL.velocity.y       = json_data["vy"];
            theDVL.velocity.z       = json_data["vz"];
            theDVL.fom              = json_data["fom"];
	        theDVL.altitude         = json_data["altitude"];
	        theDVL.velocity_valid   = json_data["velocity_valid"];
	        theDVL.status           = json_data["status"];
	        theDVL.form             = json_data["format"];
	  
            beam0.id                = json_data["transducers"][0]["id"];
            beam0.velocity          = json_data["transducers"][0]["velocity"];
            beam0.distance          = json_data["transducers"][0]["distance"];
            beam0.rssi              = json_data["transducers"][0]["rssi"];
            beam0.nsd               = json_data["transducers"][0]["nsd"];
            beam0.valid             = json_data["transducers"][0]["beam_valid"];

            beam1.id                = json_data["transducers"][1]["id"];
            beam1.velocity          = json_data["transducers"][1]["velocity"];
            beam1.distance          = json_data["transducers"][1]["distance"];
            beam1.rssi              = json_data["transducers"][1]["rssi"];
            beam1.nsd               = json_data["transducers"][1]["nsd"];
            beam1.valid             = json_data["transducers"][1]["beam_valid"];

            beam2.id                = json_data["transducers"][2]["id"];
            beam2.velocity          = json_data["transducers"][2]["velocity"];
            beam2.distance          = json_data["transducers"][2]["distance"];
            beam2.rssi              = json_data["transducers"][2]["rssi"];
            beam2.nsd               = json_data["transducers"][2]["nsd"];
            beam2.valid             = json_data["transducers"][2]["beam_valid"];

            beam3.id                = json_data["transducers"][3]["id"];
            beam3.velocity          = json_data["transducers"][3]["velocity"];
            beam3.distance          = json_data["transducers"][3]["distance"];
            beam3.rssi              = json_data["transducers"][3]["rssi"];
            beam3.nsd               = json_data["transducers"][3]["nsd"];
            beam3.valid             = json_data["transducers"][3]["beam_valid"];
            
            theDVL.beams            = {beam0, beam1, beam2, beam3};
            velocity_publisher_->publish(theDVL);

        }else{
            DVLPosEstimate.roll     = json_data["roll"];
            DVLPosEstimate.pitch    = json_data["pitch"];
            DVLPosEstimate.yaw      = json_data["yaw"];
            DVLPosEstimate.x        = json_data["x"];
            DVLPosEstimate.y        = json_data["y"];
            DVLPosEstimate.z        = json_data["z"];
            DVLPosEstimate.status   = json_data["status"];
            DVLPosEstimate.std      = json_data["std"];
            DVLPosEstimate.ts       = json_data["ts"];
            DVLPosEstimate.form     = json_data["format"];
            
            odometry_publisher_->publish(DVLPosEstimate);
        }
    }

    brov2_interfaces::msg::DVL theDVL = brov2_interfaces::msg::DVL();
    brov2_interfaces::msg::DVLOdom DVLPosEstimate = brov2_interfaces::msg::DVLOdom();
    brov2_interfaces::msg::DVLBeam beam0 = brov2_interfaces::msg::DVLBeam();
    brov2_interfaces::msg::DVLBeam beam1 = brov2_interfaces::msg::DVLBeam();
    brov2_interfaces::msg::DVLBeam beam2 = brov2_interfaces::msg::DVLBeam();
    brov2_interfaces::msg::DVLBeam beam3 = brov2_interfaces::msg::DVLBeam();    

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<brov2_interfaces::msg::DVL>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<brov2_interfaces::msg::DVLOdom>::SharedPtr odometry_publisher_;
    size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  DVLPublisher connection;
  connection.connectToDVL();

  rclcpp::spin(std::make_shared<DVLPublisher>());
  rclcpp::shutdown();
  return 0;
}