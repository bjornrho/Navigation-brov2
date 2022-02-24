# Navigation for BlueROV2

Sensor-drivers and odometry for navigation purposes using [ROS2 Galactic](https://docs.ros.org/en/ros2_documentation/galactic/index.html).

## GStreamer

Run shell scripts on PC and PI to set up 30fps live stream from camera to PC.

## Sensor suite

Sensors used in this project:

* DVL: Waterlinked A50
* IMU: Adafruit BNO055
* Barometer: Blue Robotics BAR30
* Side-scan sonar: Deepvision OSMEthernet Sonar System, 680 kHz transducers

## Barometer

Heavily based on the MS5837 library mentioned under [Libraries](#libraries).
Remember to enable I2C on the Raspberry Pi and connect on the correct pins!


## Libraries

* [BlueRobotics MS5837 Python Library](https://github.com/bluerobotics/ms5837-python)
* [Nlohman Json - JSON for Modern C++ ](https://github.com/nlohmann/json)
* [Ncurses](https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/)
* [BNO055 IMU sensor ROS2 library](https://github.com/flynneva/bno055)


## License
[MIT](https://choosealicense.com/licenses/mit/)
