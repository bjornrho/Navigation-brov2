#include "brov2_dvl/dvl_publisher_node.hpp"


int sock;


void DVLPublisher::connectToDVL(){
    struct addrinfo hints, *res;
    
    memset(&hints, '\0', sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    getaddrinfo(IP_.c_str(), port_.c_str(), &hints, &res); // Move IP and PORT values to parameter server

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

std::string DVLPublisher::getData(){
    // Adding '\0' such that only the data character is processed.
    char buf[2];
    buf[1] ='\0';
    int rec;

    std::string raw_data = "";

    while (raw_data.back() != '\n'){
        rec = recv(sock,buf,sizeof(char),0);
        if (rec == 0){
            RCLCPP_INFO(this->get_logger(), "Socket closed by the DVL, reopening");
            this->connectToDVL();
            // Add exception handling
        }
        raw_data = raw_data + buf;
    }
    return raw_data;
}

void DVLPublisher::publisher(){
    std::string raw_data = this->getData();

    nlohmann::json json_data = nlohmann::json::parse(raw_data);

    if (json_data["type"] == "velocity"){
        theDVL.time                 = json_data["time"];
	    theDVL.velocity.x           = json_data["vx"];
        theDVL.velocity.y           = json_data["vy"];
        theDVL.velocity.z           = json_data["vz"];
        theDVL.fom                  = json_data["fom"];

        for (int i = 0; i < 9; i++){
            theDVL.covariance[i]    = json_data["covariance"][i/3][i%3];}

	    theDVL.altitude             = json_data["altitude"];
	    theDVL.velocity_valid       = json_data["velocity_valid"];
	    theDVL.status               = json_data["status"];
        theDVL.time_of_validity     = json_data["time_of_validity"];
        theDVL.time_of_transmission = json_data["time_of_transmission"];
	    theDVL.form                 = json_data["format"];
        theDVL.type                 = json_data["type"];

	  
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
        theDVL.header.stamp     = now();
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
        DVLPosEstimate.type     = json_data["type"];
            
        odometry_publisher_->publish(DVLPosEstimate);
    }   
}