# Driver for Ultimate GPS
Sensor-driver for the [Ultimate GPS Breakout](https://learn.adafruit.com/adafruit-ultimate-gps) from Adafruit using UART serial connection.

## Getting Started
The driver utilizes the [Adafruit_CircuitPython_GPS module](https://github.com/adafruit/Adafruit_CircuitPython_GPS), which may be installed through:
```
pip3 install adafruit-circuitpython-gps
```

Build and source the **brov2_gps** package and all it's dependencies:
```
colcon build --packages-up-to brov2_gps
source install/setup.bash
```
Run the `gps_data_publisher` node with default parameters:
```
ros2 run brov2_gps brov2_gps_exe
```

## Node Parameters
Configuring the settings can be done through making changes to the [node parameter file](params/params.yaml) and passing this as an argument when running the `gps_data_publisher` node:
```
ros2 run brov2_gps brov2_gps_exe --ros-args --params-file src/brov2_gps/params/params.yaml
```
* **gps_topic_name**: The name of the topic where GPS readings are published; default="gps_data"
* **gps_period**: The period in (seconds) between publishing of barometer updates; default=1
* **serial_connection**: The device name of the sensor when connected to the embedded computer; default="/dev/ttyUSB1"

## GPS Interface Description
The GPS provides positional updates consisting of latitude, longitude, and altitude through the following variables:
| Variable      | Description |
| -----------   | ----------- |
| status        | Describing if the GPS has a fix receiving information: (No, Yes) = (-1, 0)            |
| latitude      | Angle in (deg) from the Equator                                                       |
| longitude     | Angle in (deg) from the Prime Meridian                                                |
| altitude      | Altitude in (meters)                                                                  | 