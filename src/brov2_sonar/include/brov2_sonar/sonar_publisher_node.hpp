// Based off sonar example from DeepVision and converted to ROS format
// Made public with permission from DeepVision. API needed


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
  Sonar{"192.168.2.62", 0xE001}
  {
    sonar_data_publisher_ = this->create_publisher<brov2_interfaces::msg::Sonar>("sonar_data", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&SonarPublisher::WriteAndPublishData, this));
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
    std::string sonar_file = "SONAR_DATA" + std::string(buffer) + ".dvs";

	DVSFile.Create(sonar_file.c_str(), leftActive, rightActive, resolution, nSamples);
  }

private:
  void ConnectToSonar();
  void WriteAndPublishData();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<brov2_interfaces::msg::Sonar>::SharedPtr sonar_data_publisher_;

  const static int BUFFERSIZE = 2048;
  float range = 30;									        // Sonar range in meters
  int nSamples = 500;									    // Number of samples per active side and ping

  int nPeriods = 32;									    // Number of periods of transmitted pulse
  float startFreq = 320000;								    // Starting frequency of transmitted pulse
  float deltaFreq = 40000;								    // Delta frequency of transmitted pulse
  bool leftActive = true;								    // true if left side is to be used
  bool rightActive = true;								    // true if right side is to be used
  float resolution = (float) ( range * 1.0 ) / nSamples;	// Resolurion of the resulting image in meters
  
  char sonarData[BUFFERSIZE];                               // Buffer to receive sonar data which is to be written to file.
  int receivedSamples;
  int pingCount;

  CDVSFileWriter DVSFile;									// The DVS File writer class
  CDSSPParser parser;
  CEthernetSonarAPI Sonar;

  brov2_interfaces::msg::Sonar sonarMsg = brov2_interfaces::msg::Sonar();
};
