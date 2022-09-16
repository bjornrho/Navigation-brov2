# Driver for A50 DVL
Sensor-driver for the [A50 DVL](https://waterlinked.github.io/dvl/dvl-a50/) from Water Linked using ethernet. Functional with the **2.1.0** and **2.2.1** software releases. 

## Getting started
The driver is dependent on the JSON library [Nlohman Json - JSON for Modern C++ ](https://github.com/nlohmann/json), which must be installed prior to using the driver. In addition, the driver is dependent on custom interfaces found in the package **brov2_interfaces**. 

*Make sure the ethernet connection to the DVL is active.* This can be done pinging the DVL from a terminal:
```
ping *insert IP of DVL*
```

Build and source the **brov2_dvl** package and all it's dependencies:
```
colcon build --packages-up-to brov2_dvl
source install/setup.bash
```
Run the `dvl_publisher` node with default parameters:
```
ros2 run brov2_dvl brov2_dvl_exe
```

## Node Parameters
Configuring the settings can be done through making changes to the [node parameter file](params/params.yaml) and passing this as an argument when running the `dvl_publisher` node:
```
ros2 run brov2_dvl brov2_dvl_exe --ros-args --params-file src/brov2_dvl/params/params.yaml
```
* **IP**: The IP address to use; default="192.168.2.52" 
* **port**: The port to use; default="16171"
* **dvl_pos_topic_name**: The name of the topic where dead reckoning updates are published; default="position_estimate"
* **dvl_vel_topic_name**: The name of the topic where velocity updates are published; default="velocity_estimate"

## DVL Interface Description
The A50 DVL provides a velocity-and-transducer report with a frequency of *2-15 Hz* depending on the altitude (vertical distance to the sea bottom or other reflecting surface). The X, Y, and Z axes are with respect to body frame of the DVL, and the report consists of the following variables:
| Variable      | Description |
| ----------- | ----------- |
| time                  | Milliseconds since last velocity report (ms)                                              |
| velocity              | Velocity in [x,y,z] directions (m/s)                                              |
| fom                   | Figure of merit, a measure of the accuracy of the velocities (m/s)                            |
| covariance            | Covariance matrix for the velocities. The *fom* is calculated from this (entries in (m/s)^2)  | altitude              | Vertical distance to the reflecting surface (m)                                                 |
| beams                 | Is a list containing information from each transducer: [id, velocity (m/s), distance (m), rssi (dBm), nsd (dBm), beam_valid (True/False)]                                            |
| velocity_valid        | If true, the DVL has a lock on the reflecting surface, and the altitude and velocities are valid (True/False) |
| status                | Reports if there are any issues with the DVL (0 for normal operation, 1 if operational issues such as high temperature)                         |
| time_of_validity      | Timestamp of the surface reflection, aka 'center of ping' (Unix timestamp in microseconds)                              |
| time_of_transmission  | Timestamp from immediately before sending of the report over TCP (Unix timestamp in microseconds)                              |
| form                  | Format type and version for this report: json_v3                                              |
| type                  | Report type: velocity                                                                         |


The A50 DVL run a dead reckoning algorithm estimating *position* and *orientation* of the DVL with respect to frame defined by the axes of the DVL's body frame. Dead reckoning updates, available at a frequency of *5 Hz*, consists of the following variables:
| Variable      | Description |
| ----------- | ----------- |
| roll      | Rotation around X axis (degrees)                                              |
| pitch     | Rotation around Y axis (degrees)                                              |
| yaw       | Rotation around Z axis, i.e. heading (degrees)                                |
| x         | Distance in X direction (m)                                                   |  
| y         | Distance in Y direction (m)                                                   |
| z         | Distance in downward direction (m)                                            |
| status    | Reports if there are any issues with the DVL (0 if no errors, 1 otherwise)    |
| std       | Standard deviation (figure of merit) for position (m)                         |
| ts        | Time stamp of report (Unix timestamp in seconds)                              |
| form      | Format type and version for this report: json_v3                              |
| type      | Report type: position_local                                                   |




