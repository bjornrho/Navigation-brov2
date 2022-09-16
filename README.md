# Navigation for BlueROV2
Sensor drivers, state estimation, trajectory generation/publishing, Text User Interface, and acoustic image processing pipeline for navigational purposes utilizing [ROS2 Galactic](https://docs.ros.org/en/ros2_documentation/galactic/index.html).

Consult separate README's for the usage of respective packages:
* [**brov2_dvl**](src/brov2_dvl/): Driver for A50 DVL
* [**brov2_barometer**](src/brov2_barometer/): Driver for Bar30 Pressure Sensor
* [**brov2_gps**](src/brov2_gps/): Driver for Ultimate GPS

## Library/Module Dependencies
The various packages rely on the following list of libraries and modules:
* [Nlohman Json - JSON for Modern C++ ](https://github.com/nlohmann/json)
* [BlueRobotics MS5837 Python Library](https://github.com/bluerobotics/ms5837-python)
* [System Management Bus (SMBus)](http://smbus.org/)
* [Adafruit_CircuitPython_GPS module](https://github.com/adafruit/Adafruit_CircuitPython_GPS)
* [Ncurses](https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/)
* [BNO055 IMU sensor ROS2 library](https://github.com/flynneva/bno055)
* [GStreamer](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html?gi-language=c)

## GStreamer Camera Stream
In order to set up 30fps live stream from camera to PC; run the shell scripts `start_video_pc.sh` and `start_video_pi.sh` on PC and PI respectively.

## Sensor suite
Sensors used in this project:

* DVL: Waterlinked A50
* IMU: Adafruit BNO055
* Barometer: Blue Robotics BAR30
* Side-scan sonar: Deepvision OSMEthernet Sonar System, 680 kHz transducers


## License
[MIT](https://choosealicense.com/licenses/mit/)
[Apache 2.0](https://choosealicense.com/licenses/apache-2.0/)

