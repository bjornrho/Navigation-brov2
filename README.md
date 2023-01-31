# Navigation for BlueROV2
Sensor drivers, state estimation, trajectory generation/publishing, Text User Interface, and acoustic image processing pipeline for navigational purposes utilizing [ROS2 Galactic](https://docs.ros.org/en/ros2_documentation/galactic/index.html). Master thesis is publicly available [here](https://hdl.handle.net/11250/3032962).

Generated Trajectory       |   Executed Trajectory
:-------------------------: | :-------------------------:
<img src="https://github.com/bjornrho/Navigation-brov2/blob/main/doc/circle_trajectory.gif" width="467" height="350" /> | <img src="https://github.com/bjornrho/Navigation-brov2/blob/main/doc/circle_MC_cropped.gif" width="305" height="350" />




Consult separate README's for the usage of respective packages:
* [**brov2_dvl**](src/brov2_dvl/): Driver for A50 DVL
* [**brov2_sonar**](src/brov2_sonar/): Driver for OSM Ethernet Sonar System
* [**brov2_barometer**](src/brov2_barometer/): Driver for Bar30 Pressure Sensor
* [**brov2_gps**](src/brov2_gps/): Driver for Ultimate GPS
* [**brov2_trajectory**](src/brov2_trajectory/): Trajectory Generator and Trajectory Publisher
* [**brov2_qekf**](src/brov2_qekf/): State Estimation using Quaternion based Error-State Kalman Filter
* [**brov2_sonar_processing**](src/brov2_sonar_processing/): Image Construction Pipeline for Side-Scan Sonar Data
* [**brov2_tui**](src/brov2_tui/): WIP

## Library/Module Dependencies
The packages/scripts rely on the following list of libraries and modules to function properly. 

**Onboard**:

* [Nlohman Json - JSON for Modern C++ ](https://github.com/nlohmann/json)
* [BlueRobotics MS5837 Python Library](https://github.com/bluerobotics/ms5837-python)
* [System Management Bus (SMBus)](http://smbus.org/)
* [Adafruit_CircuitPython_GPS module](https://github.com/adafruit/Adafruit_CircuitPython_GPS)
* [BNO055 IMU sensor ROS2 library](https://github.com/flynneva/bno055)

**Locally**:

* [Ncurses](https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/)
* [Sensor Fusion and Tracking Toolbox](https://se.mathworks.com/help/fusion/index.html?s_tid=CRUX_lftnav)

**Both**:

* [GStreamer](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html?gi-language=c)
* [NumPy](https://numpy.org/)
* [SciPy](https://scipy.org/)
* [Pandas](https://pandas.pydata.org/)
* [progressbar2](https://progressbar-2.readthedocs.io/en/latest/)

## GStreamer Camera Stream
In order to set up 30fps live stream from camera to PC; run the shell scripts `start_video_pc.sh` and `start_video_pi.sh` on PC and PI respectively.

## Hardware Configuration
The following hardware was used in this project:

* **ROV:** Blue Robotics BlueROV2 (heavy configuration)
* **Embedded Computer:** Raspberry Pi 4 Model B
* **DVL:** Waterlinked A50
* **IMU:** Adafruit BNO055
* **Barometer:** Blue Robotics BAR30
* **Side-scan sonar:** Deepvision OSMEthernet Sonar System, 680 kHz transducers


## License
[MIT](https://choosealicense.com/licenses/mit/)
[Apache 2.0](https://choosealicense.com/licenses/apache-2.0/)

