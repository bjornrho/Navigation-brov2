#include "brov2_sonar/sonar_publisher_node.hpp"



using namespace std::chrono_literals;



void SonarPublisher::ConnectToSonar(){
    receivedSamples = Sonar.GetData(sonarData, BUFFERSIZE);		// Get data from the EthernetSonar
	while (receivedSamples <= 0){
        RCLCPP_INFO(this->get_logger(), "Could not connect. Try again.\n");
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
                sonarMsg.header.stamp = now();
                // 500 is the defined sample size (must be changed in custom message)
                for (int j = 0; j < 500; j++) {
                        sonarMsg.data_zero[j] = data0[j];
                        sonarMsg.data_one[j]  = data1[j];

                sonar_data_publisher_->publish(sonarMsg);
                pingCount++;
                RCLCPP_INFO(this->get_logger(), "Complete ping # '%i' received. Publising on topic.\n", pingCount);
			    }
	        }
	    }
    }
}



