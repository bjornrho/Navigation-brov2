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
#include <boost/algorithm/string.hpp>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
int sock;
std::string oldJson = "";

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      5000ms, std::bind(&MinimalPublisher::timer_callback, this));
    }


  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

void connectToDVL(){
        struct addrinfo hints, *res;

        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        getaddrinfo("10.42.1.180", "16171", &hints, &res); // Move IP and PORT values to parameter server

        sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (connect(sock, res->ai_addr, res->ai_addrlen) < 0){
            std::cout << "Connection failed\n";
        }else{
            std::cout << "I have connected \n"; //Some try-catch error handling should be added
        }
}

std::string getData(){
    char buf[1];
    int rec;

    std::string raw_data = "";
    

    while (raw_data.back() != '\n'){
        rec = recv(sock,buf,1,0);
        if (rec == 0){
            std::cout << "Socket closed by the DVL, reopening\n";
            connectToDVL();
            // Add exception handling
        }
        raw_data = raw_data + buf;
        std::cout << raw_data;
    }

    raw_data = oldJson + raw_data + "FUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUCK";
    oldJson = "";
    // std::vector<std::string> results;
    // boost::split(results, raw_data, boost::is_any_of("\n"));
    // oldJson = raw_data[1];
    // raw_data = raw_data[0];
    return raw_data;
}


void publisher(){
    std::string data = getData();
    std::cout << data << std::endl;
}

int main(int argc, char * argv[])
{
  //rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<MinimalPublisher>());

  connectToDVL();
  while (true){
      publisher();
  }
  //rclcpp::shutdown();
  return 0;
}