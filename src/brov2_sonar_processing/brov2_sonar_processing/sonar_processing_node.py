from rclpy.node import Node
from brov2_interfaces.msg import Sonar
from brov2_interfaces.msg import DVL
from nav_msgs.msg import Odometry
from bluerov_interfaces.msg import Reference
import math
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from scipy import interpolate

from brov2_sonar_processing import side_scan_data as ssd
from brov2_sonar_processing import cubic_spline_regression as csr
from brov2_sonar_processing import plot_utils as pu

#from julia.api import Julia
#jl = Julia(compiled_modules=False)
#jl.eval('import Pkg; Pkg.activate("src/brov2_sonar_processing/brov2_sonar_processing/KnnAlgorithms");')
#jl.eval('import KnnAlgorithms')
#knn = jl.eval('KnnAlgorithms.KnnAlgorithm.knn')



class SonarProcessingNode(Node):

    def __init__(self):
        super().__init__('sonar_data_processor')
        self.sonar_subscription = self.create_subscription(Sonar, 'sonar_data', self.sonar_sub, 10)
        self.dvl_subscription   = self.create_subscription(DVL, 'dvl/velocity_estimate', self.dvl_sub, 10)
        self.state_subscription = self.create_subscription(Odometry, '/CSEI/observer/odom', self.state_sub, 10)
        
        # Julia initialization
        #self.knn = jl.import('src/brov2_sonar_processing/brov2_sonar_processing/knn.jl')
        #self.normalize_swath = jl.include('src/brov2_sonar_processing/brov2_sonar_processing/cubic_spline_regression.jl')
        self.get_logger().info("Julia functions compiled.")

        # Sonar data processing - initialization
        self.side_scan_data = ssd.side_scan_data()
        self.spline = csr.cubic_spline_regression()
        self.current_swath = ssd.swath_structure()
        self.current_altitude = 0
        self.current_state = Odometry()
        self.state_initialized = False
        #self.current_state = Reference()
        self.buffer_unprocessed_swaths = []
        self.buffer_processed_coordinate_array = []

        self.pose_csv_file_opened = False
        self.processed_frame_counter = 1
        self.ping = 0
        data_period = 0.0001
        self.timer = self.create_timer(data_period, self.run_full_pre_processing_pipeline)

        # Coefficient of variance
        self.CV_raw = []
        self.CV_normalized = []
        #self.CV_file = open("/home/bjornar/knn-test/CV_10e-1-testing.csv", "a+")
        #self.CV_writer = csv.writer(self.CV_file)
        #self.Norm_file = open("/home/bjornar/knn-test/Normalization/norm_10e-0.csv", "a+")
        #self.Norm_writer = csv.writer(self.Norm_file)
        #self.acoustic_image_file = open("/home/bjornar/knn-test/Acoustic_image_csv_files/straight_image.csv", "a+")
        #self.acoustic_image_writer = csv.writer(self.acoustic_image_file)
        self.acoustic_image_file = open("/home/bjornar/knn-test/Acoustic_image_csv_files/straight_processed_coordinate_array.csv", "a+")
        self.acoustic_image_writer = csv.writer(self.acoustic_image_file)

        # Plot related
        self.plot_utils = pu.plot_utils()
        self.raw_swath_array = np.empty((1000,1000))
        self.global_processed_swath_array = np.empty((3000,3000))
        self.raw_swath_array[:], self.global_processed_swath_array[:] = np.nan, np.nan
        self.fig, self.axs = plt.subplots(1,3)
        #self.fig, self.ax = plt.subplots()
        self.fig.show()
        #plot_period = 5  # seconds
        #self.timer = self.create_timer(plot_period, self.plot_utils.plot_acoustic_image)
    
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
        self.raw_swath_array[self.ping%1000,:500] = np.array(self.current_swath.swath_left[::-1])
        self.raw_swath_array[self.ping%1000,500:] = np.array(self.current_swath.swath_right)
        
        #if len(self.buffer_processed_coordinate_array)%100 == 0 and len(self.buffer_processed_coordinate_array) != 0:
        #    print("Plotting raw and processed data")
        #    u, v, intensity_values, frame_array, x_min, y_min, knn_intensity_mean, knn_intensity_variance, knn_filtered_image = self.construct_frame()
        #    self.buffer_processed_coordinate_array = self.buffer_processed_coordinate_array[50:]
        #    print(len(self.buffer_processed_coordinate_array))
        #    x_index = int(self.global_processed_swath_array.shape[0]/2 + x_min)
        #    y_index = int(self.global_processed_swath_array.shape[1]/2 + y_min)
        #
        #    self.global_processed_swath_array[x_index:(x_index+knn_intensity_mean.shape[0]), y_index:(y_index+knn_intensity_mean.shape[1])] = knn_intensity_mean
        #    #self.plot_utils.plot_raw_image(self.fig, self.axs[0], self.raw_swath_array)
        #    self.plot_utils.plot_raw_image(self.fig, self.axs[0], u, v, intensity_values)
        #    self.plot_utils.plot_global_batch_image(self.fig, self.axs[1], knn_intensity_mean)
        #    #self.plot_utils.plot_global_image(self.fig, self.axs[2], self.global_processed_swath_array)
        self.ping += 1
        print(self.ping)

        #normalized_swath_right, spl_right = self.spline.swath_normalization(self.current_swath.swath_right)
        #self.plot_utils.plot_CV(self.ping, self.current_swath.swath_right, normalized_swath_right, self.CV_raw, self.CV_normalized)
        #if self.ping == 1000:
        #    print("plotting")
        #    self.plot_utils.plot_raw_image(self.fig, self.ax, self.raw_swath_array)
        #    self.ping = 0

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
        theta, psi = self.pitch_yaw_from_quaternion(w, x, y, z)
        pos_x = swath_structure.state.pose.pose.position.x
        pos_y = swath_structure.state.pose.pose.position.y
        altitude = swath_structure.altitude
        # Swath definitions
        res_temp = (np.tan(self.side_scan_data.theta+self.side_scan_data.alpha/2) - np.tan(self.side_scan_data.theta-self.side_scan_data.alpha/2))
        res = res_temp*altitude/self.side_scan_data.nSamples
        number_of_samples = len(swath_structure.swath_right)

        #self.write_pose_and_swath(pos_x, pos_y, altitude, swath_structure.swath_right, swath_structure.swath_left)

        coordinate_array = np.zeros((6,number_of_samples))
        coordinate_array[4:] = np.array([swath_structure.swath_right, swath_structure.swath_left])
        #for s in range(number_of_samples):
        #    u_right = -(np.floor_divide(pos_x,res)  + np.sin(psi)*s + np.sin(theta)*np.cos(psi)*(altitude/res))
        #    v_right = (np.floor_divide(-pos_y,res)  + np.cos(psi)*s - np.sin(theta)*np.sin(psi)*(altitude/res))
        #    u_left  = -(np.floor_divide(pos_x,res)  - np.sin(psi)*s + np.sin(theta)*np.cos(psi)*(altitude/res))
        #    v_left  = (np.floor_divide(-pos_y,res)  - np.cos(psi)*s - np.sin(theta)*np.sin(psi)*(altitude/res))
        #    coordinate_array[:4,s] = np.array([u_right,u_left,v_right,v_left])

        for s in range(number_of_samples):
            x_right = -(pos_x  + np.sin(psi)*s*res + np.sin(theta)*np.cos(psi)*altitude)
            y_right = (-pos_y  + np.cos(psi)*s*res - np.sin(theta)*np.sin(psi)*altitude)
            x_left  = -(pos_x  - np.sin(psi)*s*res + np.sin(theta)*np.cos(psi)*altitude)
            y_left  = (-pos_y  - np.cos(psi)*s*res - np.sin(theta)*np.sin(psi)*altitude)
            coordinate_array[:4,s] = np.array([x_right,x_left,y_right,y_left])

        return coordinate_array

    #def construct_frame(self):
    #    u,v,intensity_values = [],[],[]
    #    for coordinate_array in self.buffer_processed_coordinate_array:
    #        u.extend(coordinate_array[0:2].flatten())
    #        v.extend(coordinate_array[2:4].flatten())               
    #        intensity_values.extend(coordinate_array[5].flatten())
    #        intensity_values.extend(coordinate_array[4].flatten())
    #    u_temp = [element * -1 for element in u]
    #    u_interval = np.linspace(min(u_temp),max(u_temp),int(max(u_temp)-min(u_temp)))
    #    v_interval = np.linspace(min(v),max(v),int(max(v)-min(v)))
    #    U,V = np.meshgrid(u_interval,v_interval)
    #
    #    linear_frame = interpolate.griddata((v,u_temp), intensity_values, (V.T,U.T), method='linear')
    #    knn_intensity_mean, knn_intensity_variance, knn_filtered_image = knn(self.side_scan_data.res, u_temp, v, intensity_values)
    #    #self.store_processed_frames(u, v, intensity_values, knn_intensity_mean, knn_filtered_image)
    #
    #    return u, v, intensity_values, linear_frame, int(min(u_temp)), int(min(v)), knn_intensity_mean, knn_intensity_variance, knn_filtered_image

    def run_full_pre_processing_pipeline(self):
        # Don't process if buffer is empty
        if len(self.buffer_unprocessed_swaths) == 0:
            return
        
        # Interpolate and construct frame if sufficient amount of swaths has arrived
        #buffer_size = len(self.buffer_processed_coordinate_array)
        #if buffer_size%100 == 0 and buffer_size != 0:
        #    frame,_,_,_,_,_,_,_,_ = self.construct_frame()
        #    self.buffer_processed_coordinate_array = self.buffer_processed_coordinate_array[50:]
        #    #self.plot_utils.img.set_data(frame)

        swath_structure = self.buffer_unprocessed_swaths[0]
        #raw_swath = swath_structure.swath_right

        #processing_frame = []
        #processing_frame.append(swath_structure.swath_right.copy())
        #processing_frame.append(swath_structure.swath_left.copy())

        # Intensity normalization
        swath_structure.swath_right,spline_right = self.spline.swath_normalization(swath_structure.swath_right)
        swath_structure.swath_left,_    = self.spline.swath_normalization(swath_structure.swath_left)    
        #processing_frame.append(swath_structure.swath_right.copy())
        #processing_frame.append(swath_structure.swath_left.copy())
        
        #self.CV_writer.writerow([np.std(raw_swath) / np.mean(raw_swath),
        #                         np.std(swath_structure.swath_right) / np.mean(swath_structure.swath_right)])
        #self.Norm_writer.writerow([raw_swath, spline_right, swath_structure.swath_right])
        #df = pd.DataFrame(np.array([raw_swath, swath_structure.altitude]))
        #df.to_csv(open('/home/bjornar/knn-test/Normalization/norm_10e-6_altitude.csv','a'), index=False, header=False)

        #self.plot_utils.plot_swath(self.ping, swath_structure.altitude, self.side_scan_data, raw_swath, spline_right)

        # Blind zone removal
        swath_structure.swath_right = self.blind_zone_removal(swath_structure.swath_right)
        swath_structure.swath_left  = self.blind_zone_removal(swath_structure.swath_left)
        #processing_frame.append(swath_structure.swath_right.copy())
        #processing_frame.append(swath_structure.swath_left.copy())

        # Slant range correction
        swath_structure = self.slant_range_correction(swath_structure)
        #processing_frame.append(swath_structure.swath_right.copy())
        #processing_frame.append(swath_structure.swath_left.copy())

        # Pose correction
        processed_coordinate_array = self.pose_correction(swath_structure)
        #processing_frame.append(processed_coordinate_array.copy())
        self.acoustic_image_writer.writerow(processed_coordinate_array)

        
        # Add element to processed and remove from unprocessed buffer
        self.buffer_processed_coordinate_array.append(processed_coordinate_array)
        self.buffer_unprocessed_swaths.pop(0)
        print(len(self.buffer_processed_coordinate_array), len(self.buffer_unprocessed_swaths))
    






    ### HELPER FUNCTIONS
    def pitch_yaw_from_quaternion(self, w, x, y, z):
        """
        Convert a quaternion into pitch and yaw
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
     
        return pitch, yaw # in radians

    def store_processed_frames(self, u, v, intensity_values, knn_intensity_mean, knn_filtered_image):
        # Storing coordinates and intensity values in csv
        raw_file_name = '~/Navigation-brov2/data/sonar/processed/raw_' + str(self.processed_frame_counter) + '.csv'
        knn_file_name = '~/Navigation-brov2/data/sonar/processed/knn_' + str(self.processed_frame_counter) + '.csv'
        knn_filtered_file_name = '~/Navigation-brov2/data/sonar/processed/knn_filtered_' + str(self.processed_frame_counter) + '.csv'
        raw_df = pd.DataFrame(list(zip(*[u, v, intensity_values]))).add_prefix("Col")
        knn_df = pd.DataFrame(knn_intensity_mean).add_prefix("Col")
        knn_filtered_df = pd.DataFrame(knn_filtered_image).add_prefix("Col")
        raw_df.to_csv(raw_file_name, index=False)
        knn_df.to_csv(knn_file_name, index=False)
        knn_filtered_df.to_csv(knn_filtered_file_name, index=False)
        print("Frame #", self.processed_frame_counter, " stored.")
        self.processed_frame_counter += 1

    def write_pose_and_swath(self, pos_x, pos_y, altitude, swath_right, swath_left):
        if not self.pose_csv_file_opened:
            pose_and_swath_file_name = '/home/bjornar/knn-test/start_turn_pose_and_swath.csv'
            self.pose_and_swath_file = open(pose_and_swath_file_name, "a+")
        self.pose_and_swath_writer = csv.writer(self.pose_and_swath_file)
        self.pose_and_swath_writer.writerow([pos_x, pos_y, altitude, swath_right, swath_left])