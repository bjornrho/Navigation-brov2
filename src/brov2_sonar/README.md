# Driver for OSM Ethernet Sonar System
Sensor-driver for the [OSM Ethernet Sonar System](https://deepvision.se/products/oem-sonar-modules/) from Deep Vision.

## Getting started
The driver utilizes an API from Deep Vision which is not publicly available in this repository due to copyrights. In addition, the driver is dependent on custom interfaces found in the package **brov2_interfaces**.

Build and source the **brov2_sonar** package and all it's dependencies:
```
colcon build --packages-up-to brov2_sonar
source install/setup.bash
```
Run the `sonar_publisher` node with default parameters:
```
ros2 run brov2_sonar brov2_sonar_exe
```

## Node Parameters
Configuring the settings can be done through making changes to the [node parameter file](params/params.yaml) and passing this as an argument when running the `sonar_publisher` node:
```
ros2 run brov2_sonar brov2_sonar_exe --ros-args --params-file src/brov2_sonar/params/params.yaml
```
* **sonar_topic_name**: The name of the topic where pressure sensor readings are published; default="barometer_data"
* **sonar_range**: Sonar range in (meters), the expected distance to the sea floor during a mission; default=30
* **sonar_number_of_samples**: Number of samples per active side and ping. Should coincide with pipeline and custom interface; default=500
* **sonar_left_active**: Bool activating the left side transducer; default=true
* **sonar_right_active**: Bool activating the right side transducer; default=true

## Sonar Interface Description
The OSM Ethernet Sonar System provides single scan lines through populating a custom interface consisting of the following variables:
| Variable      | Description |
| -----------   | ----------- |
| header                          | Header containing time stamp information for the single scan line       |
| byte[nSample] data_zero         | Data from the left transducer if active.                                |
| byte[nSample] data_one          | Data from the right transducer if active.                               |

*NOTE: The nSample hardcoded in the brov2_interface sonar message **must** coincide with the defined sonar_number_of_samples parameter.*