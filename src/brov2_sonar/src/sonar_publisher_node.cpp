#include "brov2_sonar/sonar_publisher_node.hpp"



using namespace std::chrono_literals;



void SonarPublisher::ConnectToSonar(){
    receivedSamples = Sonar.GetData(sonarData, BUFFERSIZE);		// Get data from the EthernetSonar
	while (receivedSamples <= 0){
        std::cout << "try again\n";
		Sonar.StartRec(range);
    	usleep(50000);
    	receivedSamples = Sonar.GetData(sonarData, BUFFERSIZE);
    	usleep(50000);
    	}

	std::cout << "Sonar connected successfully.\n";
}


void SonarPublisher::WriteAndPublishData(){
    receivedSamples = Sonar.GetData(sonarData, BUFFERSIZE);	    // Get data from the EthernetSonar
	if (receivedSamples > 0){
		for (int i = 0; i < receivedSamples; i++){			// Parse the incoming data stream from TCP
			// Add the received data to the parser.
			// The parser.Add() will return true when a complete ping has been added.
			if (parser.Add(sonarData[i])){
				// A complete ping has been added to the parser. We will add it to the
				// .dvs file using the DVSFileWriter.
				char* data0;
				char* data1;
				int size0 = 0;
				int size1 = 0;

				// Get the sonar data from the parser
				parser.GetChannelData(data0, &size0, data1, &size1); 

				// Add the sonar data to the file, skipping positioning data for simplicity
				DVSFile.AddPingData(0.0, 0.0, 0.0, 0.0, data0, size0, data1, size1);

                // Building sonar interface and publishing
                
                sonarMsg.left_data.data = std::string(data0);//, size0);
                sonarMsg.right_data.data = std::string(data1);//, size1);

                sonar_data_publisher_->publish(sonarMsg);
                pingCount++;
                RCLCPP_INFO(this->get_logger(), "Complete ping # '%i' received. Publising on topic.\n", pingCount);
				// std::cout << "Complete ping #" << pingCount << " received. Publising on topic.\n";
			}
	    }
	}
}




