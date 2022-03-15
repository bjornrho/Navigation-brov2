from rclpy.node import Node
from brov2_interfaces.msg import Sonar
import numpy as np
import matplotlib.pyplot as plt

from brov2_sonar_processing import side_scan_data as ssd
from brov2_sonar_processing import cubic_spline_regression as csr
#import julia



class SonarProcessingNode(Node):

    def __init__(self):
        super().__init__('sonar_data_processor')
        self.subscription = self.create_subscription(
            Sonar,
            'sonar_data',
            self.sonar_sub,
            10)
        self.subscription  # prevent unused variable warning

        # # Julia initialization
        # jl = julia.Julia(compiled_modules=False)
        # self.normalize_swath = jl.include('src/brov2_sonar_processing/brov2_sonar_processing/cubic_spline_regression.jl')
        # self.get_logger().info("Julia function compiled.")

        # Sonar data object
        self.side_scan_data = ssd.side_scan_data()
        self.spline = csr.cubic_spline_regression()

        # Plot related
        self.fig, self.axs = plt.subplots(1)
        self.fig.show()
        self.img_array = np.zeros((500,1000))
        self.img = plt.imshow(self.img_array,cmap='gray',vmin=0,vmax=2, interpolation=None)

    def plot_swath(self, ping, swath_right, spline_right, swath_left=None, spline_left=None):
        plt.cla()
        if swath_left is None:
            # Plotting raw swath and spline fitted curve
            self.axs.plot(range(0,self.side_scan_data.nSamples), swath_right, color='blue')
            self.axs.plot(range(0,self.side_scan_data.nSamples), spline_right, color='orange')
            self.axs.legend(["Across-track signal","Cubic spline fitted curve"], loc="upper right")
        else:
            self.axs.plot(range(-self.side_scan_data.nSamples,0), swath_left[::-1], color='blue')
            self.axs.plot(range(-self.side_scan_data.nSamples,0), spline_left[::-1],
                          range(0,self.side_scan_data.nSamples),  spline_right, color='orange', zorder=3)
            self.axs.plot(range(0,self.side_scan_data.nSamples),  swath_right, color='blue')
            self.axs.axvline(x=0, ymin=0, color='black', linestyle='dotted')

            self.axs.legend(["Across-track signal", "Cubic spline fitted curve"], loc="upper right")
            

        self.axs.set(xlabel='# of sample per ping', ylabel='Ping return (log compressed)')
        self.axs.set_title("Ping %i" % ping)
        plt.gca().axis('tight')

        plt.pause(10e-5)
        self.fig.canvas.draw()

    def plot_acoustic_image(self, ping, swath_right, swath_left):
        self.img_array[ping%500,:500] = np.array(swath_left[::-1])
        self.img_array[ping%500,500:] = np.array(swath_right)
        self.img.set_data(self.img_array)
        
        plt.xlabel('Across track') 
        plt.ylabel('Along track')
        plt.title("Acoustic Image")

        # Limiting refresh rate due to time consumption
        if ping%25 == 0:
            plt.pause(10e-5)
            self.fig.canvas.draw()

    def sonar_sub(self, msg):
        # Right transducer data handling
        transducer_raw_right = msg.data_zero
        swath_right = [int.from_bytes(byte_val, "big") for byte_val in transducer_raw_right]
        normalized_swath_right, spl_right = self.spline.swath_normalization(swath_right)
        self.side_scan_data.right.append(np.array(normalized_swath_right))

        # Left transducer data handling
        transducer_raw_left = msg.data_one
        swath_left = [int.from_bytes(byte_val, "big") for byte_val in transducer_raw_left]
        normalized_swath_left, spl_left = self.spline.swath_normalization(swath_left)
        self.side_scan_data.left.append(np.array(normalized_swath_left))

        ping = len(self.side_scan_data.right)

        self.plot_acoustic_image(ping, normalized_swath_right, normalized_swath_left)
        #self.plot_swath(ping, swath_right, spl_right, swath_left, spl_left)
        self.get_logger().info("Ping #%i" % ping)
        