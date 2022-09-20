# Trajectory Generation and Publishing 
MATLAB scripts to help generate lawnmower trajectories and ROS2 package to publish the trajectories.

## Trajectory Publisher - Getting Started
The driver utilizes [NumPy](https://numpy.org/) and [progressbar2](https://progressbar-2.readthedocs.io/en/latest/), which both may be installed using [pip](https://pypi.org/project/pip/):
```
pip install numpy
pip install progressbar2
```

In addition, the driver is dependent on custom interfaces found in the package **bluerov_interfaces** and **brov2_interfaces**. *Two interface packages are used, since trajectory interface must be the same as utilized by the controller.*

Build and source the **brov2_trajectory** package and all it's dependencies:
```
colcon build --packages-up-to brov2_trajectory
source install/setup.bash
```
Run the `trajectory_publisher` node with default parameters:
```
ros2 run brov2_trajectory brov2_trajectory_exe
```

## Node Parameters
Configuring the settings can be done through making changes to the [node parameter file](params/params.yaml) and passing this as an argument when running the `trajectory_publisher` node:
```
ros2 run brov2_trajectory brov2_trajectory_exe --ros-args --params-file src/brov2_trajectory/params/params.yaml
```
* **trajectory_topic_name**: The name of the topic where pressure sensor readings are published; default="barometer_data"
* **trajectory_file_name**: The namce of the csv-file containing the trajectory to be published; default="MC_circle.csv"
* **trajectory_period**: The period in (seconds) between publishing set-point from trajectory; default=0.01
* **update_printout**: Bool allowing printout of current position and yaw set-point; default=True


## Trajectory Interface Description
A trajectory consists of and is published as a consecutive row of separate set-points. Each set-point is described using the custom *Reference.msg* interface from **bluerov_interface** consisting of the following variables:
| Variable      | Description |
| -----------   | ----------- |
| pos           | The reference position [x,y,z] in the semi-global frame defined upon initialization of QEKF   |
| quat          | The reference orientation in quaternions [w,x,y,z]                                            |
| velocity      | The reference velocity (linear and angular)                                                   |
| acceleration  | The reference acceleration (linear and angular)                                               |  