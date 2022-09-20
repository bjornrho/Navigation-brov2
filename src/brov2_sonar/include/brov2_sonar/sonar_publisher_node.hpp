// Based off of sonar example from DeepVision and converted to be utilized in ROS framework
// Made public with permission from DeepVision. API needed.


#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <string>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "brov2_interfaces/msg/sonar.hpp"

#include "EthernetSonarAPI.hpp"
#include "DVSFileWriter.hpp"
#include "DSSPParser.hpp"

using namespace std::chrono_literals;


class SonarPublisher : public rclcpp::Node
{
public:
  SonarPublisher()
  : Node("sonar_publisher"),
  Sonar{"192.168.2.62", 0xE001} // CHANGE HARDCODED IP IF NECESSARY
  { this->declare_parameter<std::string>("sonar_topic_name", "sonar_data");
    this->declare_parameter<float>("sonar_range", 30);
    this->declare_parameter<int>("sonar_number_of_samples", 500);
    this->declare_parameter<bool>("sonar_left_active", true);
    this->declare_parameter<bool>("sonar_right_active", true);

    this->get_parameter("sonar_topic_name", sonar_topic_name_);
    this->get_parameter("sonar_range", range);
    this->get_parameter("sonar_number_of_samples", nSamples);
    this->get_parameter("sonar_left_active", leftActive);
    this->get_parameter("sonar_right_active", rightActive);

    std::chrono::duration<long int> publish_period = std::chrono::seconds(long(2*range/1500.0));

    sonar_data_publisher_ = this->create_publisher<brov2_interfaces::msg::Sonar>(sonar_topic_name_, 10);
    timer_ = this->create_wall_timer(publish_period, std::bind(&SonarPublisher::WriteAndPublishData, this));
    // Initialize ping count
    pingCount = 0;
    // Set up the pulse characteristics.
	Sonar.DSSP_SetPulseDual(nPeriods, startFreq, deltaFreq, nPeriods, startFreq, deltaFreq);
	// Set up the sampling characteristics.
	Sonar.DSSP_SetSampling(nSamples, leftActive, rightActive, false);
	// Start the recording.
	Sonar.StartRec(range);
    // Connect to the Sonar
    ConnectToSonar();
    // Set up the file writer with the filename "SONAR_DATA_*today*.dvs" and the settings we have sent to the sonar.
    time_t now;
    time(&now);
    char buffer[21];
    strftime(buffer, 21, "%F%H%M%S", localtime(&now));
    std::string sonar_file = "SONAR_DATA_" + std::string(buffer) + ".dvs";

	DVSFile.Create(sonar_file.c_str(), leftActive, rightActive, resolution, nSamples);
  }

private:
  void ConnectToSonar();
  void WriteAndPublishData();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<brov2_interfaces::msg::Sonar>::SharedPtr sonar_data_publisher_;

  std::string sonar_topic_name_;

  // 1. for 670k Transducers
  // you can use 32 periods, df=50k (645k to 685k)
  // If shorter ranges you can use 16 periods and longer ranges try 64 periods.

  const static int BUFFERSIZE = 2048;
  float range;									            // Sonar range in meters
  int nSamples;									            // Number of samples per active side and ping. Must coincide with sonar interface message.

  int nPeriods = 32;									    // Number of periods of transmitted pulse
  float startFreq = 645000;//320000;					    // Starting frequency of transmitted pulse
  float deltaFreq = 50000; //40000;		    			    // Delta frequency of transmitted pulse
  bool leftActive;      								    // true if left side is to be used
  bool rightActive;     								    // true if right side is to be used
  float resolution = (float) ( range * 1.0 ) / nSamples;	// Resolution of the resulting image in meters
  
  char sonarData[BUFFERSIZE];                               // Buffer to receive sonar data which is to be written to file.
  int receivedSamples;
  int pingCount;

  CDVSFileWriter DVSFile;									// The DVS File writer class
  CDSSPParser parser;
  CEthernetSonarAPI Sonar;

  brov2_interfaces::msg::Sonar sonarMsg = brov2_interfaces::msg::Sonar();
};
