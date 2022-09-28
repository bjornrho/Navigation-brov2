# State Estimation using Quaternion based Error-State Kalman Filter
State estimation through implementation of [Joan Solà's formulation](https://arxiv.org/abs/1711.02508) of a quaternion based error-state Kalman Filter. The filter fuses *linear velocity* measurements from the DVL, *depth* measurements from the pressure sensor, *linear acceleration* and *angular velocity* measurements in addition to *orientation estimates* from the IMU.

<p align="center">
  <img src="https://github.com/bjornrho/Navigation-brov2/blob/main/doc/qekf_block_diagram_dark.svg#gh-dark-mode-only"/>
  <img src="https://github.com/bjornrho/Navigation-brov2/blob/main/doc/qekf_block_diagram.svg#gh-light-mode-only"/>
</p>


## Getting Started
The driver utilizes [NumPy](https://numpy.org/), which may be installed using [pip](https://pypi.org/project/pip/):
```
pip install numpy
```
In addition, the driver is dependent on custom interfaces found in the package **bluerov_interfaces** and **brov2_interfaces**. *Two interface packages are used, since trajectory interface must be the same as utilized by the controller.*

Build and source the **brov2_qekf** package and all it's dependencies:
```
colcon build --packages-up-to brov2_qekf
source install/setup.bash
```
Run the `qekf_state_estimator` node with default parameters:
```
ros2 run brov2_qekf brov2_qekf_exe
```

## Node Parameters
Configuring the settings can be done through making changes to the [node parameter file](params/params.yaml) and passing this as an argument when running the `qekf_state_estimator` node:
```
ros2 run brov2_qekf brov2_qekf_exe --ros-args --params-file src/brov2_qekf/params/params.yaml
```
* **x_0**: Initial values of nominal state (see thesis for filter definition); default=[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
* **std_a**: Linear acceleration measurement standard deviation noise (meters/seconds^2); default=0.69782
* **std_gyro**: Angular velocity measurement standard deviation noise (radians/seconds); default=0.011393
* **std_dvl**: Linear velocity measurement standard deviation noise (meters/seconds); default=0.01
* **std_depth**: Depth measurement standard deviation noise (meters); default=0.0024678
* **std_orientation**: Orientation estimate standard deviation noise (radians); default=0.00091673
* **std_a_bias**: Linear acceleration bias standard deviation noise (meters/(seconds^2 √seconds); default=0.21396
* **std_gyro_bias**: Angular velocity bias standard deviation noise (radians/(seconds √seconds); default=0.014258
* **dvl_offset**: Physical offset of DVL origo from body frame origo [x,y,z] in (meters); default=[-0.020, -0.095, 0.133]
* **barometer_offset**: Physical offset of pressure sensor origo from body frame origo [x,y,z] in (meters); default=[-0.175, -0.015, -0.05]
* **imu_offset**: Physical offset of IMU origo from body frame origo [x,y,z] in (meters); default=[0.057, 0.027, -0.025]
* **dvl_vel_topic_name**: The name of the topic where velocity updates are published; default="dvl/velocity_estimate"
* **imu_topic_name**: The name of the topic where imu updates are published; default="bno055/imu"
* **barometer_topic_name**: The name of the topic where pressure sensor readings are published; default="barometer/barometer_data"
* **state_estimate_topic_name**: The name of the topic where state estimates are published; default="/CSEI/observer/odom"

The default values are a combination of values found through experimental measurements and the respective sensor data sheet, whichever proved to be the largest. See thesis for further explanation.

*Note that the A50 DVL provides a covariance matrix accompanying velocity measurements. Hence, **std_dvl** is not actually used, but it is added for the sake of completeness.*

## Node Services
The following three services have been implemented to make the filter easier to use and analyze in real-life scenarios:
| Service      | Description |
| ----------- | ----------- |
| /brov2_qekf/set_yaw_offset            | Storing yaw offset value making the current yaw value 'zero'.     |
| /brov2_qekf/reset_qekf                | Re-initializing the filter without the need to kill and start the `qekf_state_estimator` node again.|
| /brov2_qekf/start_stop_NIS_logging    | Start and stop logging of Normalized Innovation Squared (NIS). Logged files can be found in the *./brov2_qekf/NIS_values/* directory.  |

Run a service using:
```
ros2 service call *insert service* std_srvs/srv/Trigger
``` 