# Driver for Bar30 Pressure Sensor
Sensor-driver for the [Bar30 Pressure Sensor](https://bluerobotics.com/store/sensors-sonars-cameras/sensors/bar30-sensor-r1/) from Blue Robotics using SMBus.

## Getting Started
The driver utilizes the [ms5837 python module](https://github.com/bluerobotics/ms5837-python) and is dependent on the System Management Bus [SMBus](http://smbus.org/), which may be installed through:
```
sudo apt-get install python-smbus
```
In addition, the driver is dependent on custom interfaces found in the package **brov2_interfaces**.

*Remember to enable the I2C interface on the embedded computer used.*

Build and source the **brov2_barometer** package and all it's dependencies:
```
colcon build --packages-up-to brov2_barometer
source install/setup.bash
```
Run the `barometer_data_publisher` node with default parameters:
```
ros2 run brov2_barometer brov2_barometer_exe
```

## Node Parameters
Configuring the settings can be done through making changes to the [node parameter file](params/params.yaml) and passing this as an argument when running the `barometer_data_publisher` node:
```
ros2 run brov2_barometer brov2_barometer_exe --ros-args --params-file src/brov2_barometer/params/params.yaml
```
* **barometer_topic_name**: The name of the topic where pressure sensor readings are published; default="barometer_data"
* **barometer_period**: The period in (seconds) between publishing of barometer updates; default=0.1
* **fluid_density**: The fluid density in (kg/m^3) of the fluid for depth measurements. Use 997 for fresh water. Use 1029 for salt water; default=1029

## Barometer Interface Description
The Bar30 pressure sensor provides measurement updates consisting of depth, temperature, and pressure through the following variables:
| Variable      | Description |
| -----------   | ----------- |
| depth                     | Depth measured in (meters)                |
| pressure_mbar             | Pressure measured in (mbar)               |
| pressure_psi              | Pressure measured in (psi)                |
| temperature_celsius       | Temperature measured in (celsius)         |  
| temperature_farenheit     | Temperature measured in (farenheit)       |