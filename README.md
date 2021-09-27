# [Navigation for Blue ROV2](https://www.youtube.com/watch?v=dQw4w9WgXcQ)

Sensor-drivers and odometry for navigation purposes using [ROS2 Galactic](https://docs.ros.org/en/ros2_documentation/galactic/index.html).


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


## License
[MIT](https://choosealicense.com/licenses/mit/)
