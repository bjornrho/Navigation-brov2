# Image Construction Pipeline for Side-Scan Sonar Data
Processing scan lines from side-scan sonar through an image construction pipeline. The pipeline performs echo intensity
normalization, blind zone removal, slant range correction, and geometric correction, in addition to interpolation and filtering. See the thesis for details.

<p align="center">
  <img src="https://github.com/bjornrho/Navigation-brov2/blob/main/doc/image_construction_pipeline.svg"/>
</p>

## Getting Started
The driver utilizes [NumPy](https://numpy.org/), [SciPy](https://scipy.org/), and [Pandas](https://pandas.pydata.org/), which may be installed using [pip](https://pypi.org/project/pip/):
```
pip install numpy
pip install scipy
pip install pandas
```
The package is written in Python, however Julia must be installed to perform part of the processing. In addition, the driver is dependent on custom interfaces found in the package **brov2_interfaces**.

Build and source the **brov2_sonar_processing** package and all it's dependencies:
```
colcon build --packages-up-to brov2_sonar_processing
source install/setup.bash
```
Run the `sonar_data_processor` node with default parameters:
```
ros2 run brov2_sonar_processing brov2_sonar_processing_exe
```

In the *./processed_frames/* directory three different data frames are stored as .csv-files when running the processing pipeline:
* *raw*: This is the processed frame without any interpolation, consisting of image coordinates [u,v] and intensity values.
* *knn*: This is the processed frame with knn-interpolation.
* *knn_filtered*: This is the processed frame with knn-interpolation and anisotropic diffusion filtering.

## Node Parameters
Configuring the settings can be done through making changes to the [node parameter file](params/params.yaml) and passing this as an argument when running the `sonar_data_processor` node:
```
ros2 run brov2_sonar_processing brov2_sonar_processing_exe --ros-args --params-file src/brov2_sonar_processing/params/params.yaml
```
* **sonar_data_topic_name**: The name of the topic where raw sonar data is published; default="sonar_data"
* **dvl_vel_topic_name**: The name of the topic where velocity updates are published; default="dvl/velocity_estimate"
* **qekf_state_estimate_topic_name**: The name of the topic where state estimates are published; default="/CSEI/observer/odom"
* **scan_lines_per_stored_frame**: The number of scan lines to process per stored frame; default=100
* **processing_period**: The period in (seconds) between processing scan-lines (should match the ); default=0.0001
* **number_of_samples_sonar**: Number of samples per active side and ping of the sonar. Should coincide with sonar driver and custom interface; default=500
* **range_sonar**: Sonar range in (meters), the expected distance to the sea floor during a mission; default=30

*Note, the processing_period should be set equal to (or shorter) than the publishing period of the **brov2_sonar** sonar driver to avoid pile-up of unprocessed data.*


## Interpolation and filtering
The various interpolation and filtering employed can be tweaked through numerous parameters. The current settings are a result of limited testing due to time constraints. Hence, the user is advised to test different parameters in the *knn.jl* script, as well as various interpolation techniques using SciPy's griddata() function in *sonar_processing_node.py*.