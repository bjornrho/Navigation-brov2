import sys
sys.path.append('utility_functions')
import utility_functions

from rclpy.node import Node
from brov2_interfaces.msg import Sonar
from brov2_interfaces.msg import DVL
from nav_msgs.msg import Odometry
import math
import numpy as np
import pandas as pd
from scipy import interpolate

from brov2_sonar_processing import side_scan_data as ssd
from brov2_sonar_processing import cubic_spline_regression as csr
from brov2_sonar_processing import plot_utils as pu

from julia.api import Julia
jl = Julia(compiled_modules=False)
jl.eval('import Pkg; Pkg.activate("src/brov2_sonar_processing/brov2_sonar_processing/KnnAlgorithms");')
jl.eval('import KnnAlgorithms')
knn = jl.eval('KnnAlgorithms.KnnAlgorithm.knn')



class SonarProcessingNode(Node):

    def __init__(self):
        super().__init__('sonar_data_processor')
        self.declare_parameters(namespace='',
                                parameters=[('sonar_data_topic_name', 'sonar_data'),
                                            ('dvl_vel_topic_name', 'dvl/velocity_estimate'),
                                            ('qekf_state_estimate_topic_name', '/CSEI/observer/odom'),
                                            ('processing_period', 0.0001),
                                            ('number_of_samples_sonar', 500),
                                            ('range_sonar', 30)])
        sonar_data_topic_name, dvl_vel_topic_name = self.get_parameters(['sonar_data_topic_name', 'dvl_vel_topic_name'])
        qekf_state_estimate_topic_name = self.get_parameter('qekf_state_estimate_topic_name')
        processing_period, number_of_samples_sonar, range_sonar = self.get_parameters(['processing_period',
                                                                                       'number_of_samples_sonar',
                                                                                       'range_sonar'])


        self.sonar_subscription = self.create_subscription(Sonar, sonar_data_topic_name.value, self.sonar_sub, 10)
        self.dvl_subscription   = self.create_subscription(DVL, dvl_vel_topic_name.value, self.dvl_sub, 10)
        self.state_subscription = self.create_subscription(Odometry, qekf_state_estimate_topic_name.value, self.state_sub, 10)

        # Sonar data processing - initialization
        self.side_scan_data = ssd.side_scan_data(number_of_samples_sonar.value, range_sonar.value)
        self.spline = csr.cubic_spline_regression()
        self.current_swath = ssd.swath_structure()
        self.current_altitude = 0
        self.current_state = Odometry()
        self.state_initialized = False
        self.buffer_unprocessed_swaths = []
        self.buffer_processed_coordinate_array = []

        self.processed_frame_counter = 1
        self.timer = self.create_timer(processing_period.value, self.run_full_pre_processing_pipeline)

        self.get_logger().info("Sonar processing node initialized.")

    
    ### SENSOR AND STATE SUBSCRIPTION FUNCTIONS
    def sonar_sub(self, sonar_msg):
        if not self.state_initialized:
            return
        # Right transducer data handling
        transducer_raw_right = sonar_msg.data_zero
        self.current_swath.swath_right = [int.from_bytes(byte_val, "big") for byte_val in transducer_raw_right] # Big endian
        # Left transducer data handling
        transducer_raw_left = sonar_msg.data_one
        self.current_swath.swath_left = [int.from_bytes(byte_val, "big") for byte_val in transducer_raw_left]
        # Adding related state and altitude of platform for processing purpose
        self.current_swath.state, self.current_swath.altitude = self.current_state, self.current_altitude
        # Append to the buffer of unprocessed swaths and array for plotting
        self.buffer_unprocessed_swaths.append(self.current_swath)


    def dvl_sub(self, dvl_msg):
        # Altitude values of -1 are invalid
        if dvl_msg.altitude != -1:
            self.current_altitude = dvl_msg.altitude

    def state_sub(self, state_msg):
        self.current_state = state_msg
        self.state_initialized = True



    ### DATA PROCESSING FUNCTIONS
    def blind_zone_removal(self, swath):
        r_FBR = self.current_altitude / np.sin(self.side_scan_data.theta + self.side_scan_data.alpha/2)
        index_FBR = int(np.floor_divide(r_FBR, self.side_scan_data.res))
        swath[:index_FBR] = [np.nan] * index_FBR
        return swath

    def slant_range_correction(self, swath_structure):
        # Variation of Burguera et al. 2016, Algorithm 1
        altitude = swath_structure.altitude
        res = self.side_scan_data.res
        number_of_samples = len(swath_structure.swath_right)
        corrected_swath_right, corrected_swath_left = [],[]
        for sample in range(number_of_samples):
            exact_bin = np.sqrt((res*sample)**2 + altitude**2)/res
            floor_bin = int(min(math.floor(exact_bin), number_of_samples-1))
            ceiling_bin = int(min(math.ceil(exact_bin), number_of_samples-1))
            weight_1 = exact_bin - floor_bin
            weight_2 = 1 - weight_1
            
            corrected_intensity_value_right = weight_2*swath_structure.swath_right[floor_bin] + weight_1*swath_structure.swath_right[ceiling_bin]
            corrected_intensity_value_left = weight_2*swath_structure.swath_left[floor_bin] + weight_1*swath_structure.swath_left[ceiling_bin]
            corrected_swath_right.append(corrected_intensity_value_right)
            corrected_swath_left.append(corrected_intensity_value_left)
        
        swath_structure.swath_right, swath_structure.swath_left = corrected_swath_right, corrected_swath_left
        return swath_structure


    def pose_correction(self, swath_structure):
        # State related extraction
        [w,x,y,z] = [swath_structure.state.pose.pose.orientation.w, swath_structure.state.pose.pose.orientation.x, 
                     swath_structure.state.pose.pose.orientation.y, swath_structure.state.pose.pose.orientation.z]
        theta, psi = utility_functions.pitch_yaw_from_quaternion(w, x, y, z)
        pos_x = swath_structure.state.pose.pose.position.x
        pos_y = swath_structure.state.pose.pose.position.y
        altitude = swath_structure.altitude
        # Swath definitions
        res_temp = (np.tan(self.side_scan_data.theta+self.side_scan_data.alpha/2) - np.tan(self.side_scan_data.theta-self.side_scan_data.alpha/2))
        res = res_temp*altitude/self.side_scan_data.nSamples
        number_of_samples = len(swath_structure.swath_right)

        coordinate_array = np.zeros((6,number_of_samples))
        coordinate_array[4:] = np.array([swath_structure.swath_right, swath_structure.swath_left])

        for s in range(number_of_samples):
            x_right = -(pos_x  + np.sin(psi)*s*res + np.sin(theta)*np.cos(psi)*altitude)
            y_right = (-pos_y  + np.cos(psi)*s*res - np.sin(theta)*np.sin(psi)*altitude)
            x_left  = -(pos_x  - np.sin(psi)*s*res + np.sin(theta)*np.cos(psi)*altitude)
            y_left  = (-pos_y  - np.cos(psi)*s*res - np.sin(theta)*np.sin(psi)*altitude)
            coordinate_array[:4,s] = np.array([x_right,x_left,y_right,y_left])

        return coordinate_array

    def construct_frame(self):
        u,v,intensity_values = [],[],[]
        for coordinate_array in self.buffer_processed_coordinate_array:
            u.extend(coordinate_array[0:2].flatten())
            v.extend(coordinate_array[2:4].flatten())               
            intensity_values.extend(coordinate_array[5].flatten())
            intensity_values.extend(coordinate_array[4].flatten())
        u_temp = [element * -1 for element in u]
        u_interval = np.linspace(min(u_temp),max(u_temp),int(max(u_temp)-min(u_temp)))
        v_interval = np.linspace(min(v),max(v),int(max(v)-min(v)))
        U,V = np.meshgrid(u_interval,v_interval)

        ### Attempt various methods [linear, nearest, spline] using griddata if desired ###
        linear_frame = interpolate.griddata((v,u_temp), intensity_values, (V.T,U.T), method='linear')

        # Or Knn method as described in hogstad2022sidescansonar
        knn_intensity_mean, knn_intensity_variance, knn_filtered_image = knn(self.side_scan_data.res, u_temp, v, intensity_values)
        self.store_processed_frames(u, v, intensity_values, knn_intensity_mean, knn_filtered_image)
    
        return u, v, intensity_values, linear_frame, int(min(u_temp)), int(min(v)), knn_intensity_mean, knn_intensity_variance, knn_filtered_image

    def run_full_pre_processing_pipeline(self):
        # Don't process if buffer is empty
        if len(self.buffer_unprocessed_swaths) == 0:
            return
        
        # Interpolate and construct frame if sufficient amount of swaths has arrived
        buffer_size = len(self.buffer_processed_coordinate_array)
        if buffer_size%100 == 0 and buffer_size != 0:
            _,_,_,_,_,_,_,_,_ = self.construct_frame()
            self.buffer_processed_coordinate_array = self.buffer_processed_coordinate_array[50:]

        swath_structure = self.buffer_unprocessed_swaths[0]

        # Intensity normalization
        swath_structure.swath_right,_ = self.spline.swath_normalization(swath_structure.swath_right)
        swath_structure.swath_left,_    = self.spline.swath_normalization(swath_structure.swath_left)

        # Blind zone removal
        swath_structure.swath_right = self.blind_zone_removal(swath_structure.swath_right)
        swath_structure.swath_left  = self.blind_zone_removal(swath_structure.swath_left)

        # Slant range correction
        swath_structure = self.slant_range_correction(swath_structure)

        # Pose correction
        processed_coordinate_array = self.pose_correction(swath_structure)

        # Add element to processed and remove from unprocessed buffer
        self.buffer_processed_coordinate_array.append(processed_coordinate_array)
        self.buffer_unprocessed_swaths.pop(0)


    def store_processed_frames(self, u, v, intensity_values, knn_intensity_mean, knn_filtered_image):
        # Storing coordinates and intensity values in csv
        raw_file_name = 'src/brov2_sonar_processing/processed_frames/raw_' + str(self.processed_frame_counter) + '.csv'
        knn_file_name = 'src/brov2_sonar_processing/processed_frames/knn_' + str(self.processed_frame_counter) + '.csv'
        knn_filtered_file_name = 'src/brov2_sonar_processing/processed_frames/knn_filtered_' + str(self.processed_frame_counter) + '.csv'
        
        raw_df = pd.DataFrame(list(zip(*[u, v, intensity_values]))).add_prefix("Col")
        knn_df = pd.DataFrame(knn_intensity_mean).add_prefix("Col")
        knn_filtered_df = pd.DataFrame(knn_filtered_image).add_prefix("Col")
        
        raw_df.to_csv(raw_file_name, index=False)
        knn_df.to_csv(knn_file_name, index=False)
        knn_filtered_df.to_csv(knn_filtered_file_name, index=False)
        
        print("Frame #" + str(self.processed_frame_counter) + " stored.")
        self.processed_frame_counter += 1
