from rclpy.node import Node
from brov2_interfaces.msg import Sonar
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from brov2_sonar_processing import side_scan_data as ssd


class SonarProcessingNode(Node):

    def __init__(self):
        super().__init__('sonar_data_processor')
        self.subscription = self.create_subscription(
            Sonar,
            'sonar_data',
            self.sonar_sub,
            10)
        self.subscription  # prevent unused variable warning

        # Sonar data object
        self.side_scan_data = ssd.side_scan_data()
        self.swath_right = np.zeros((self.side_scan_data.nSamples,0)).tolist()
        self.swath_left = np.zeros((self.side_scan_data.nSamples,0)).tolist()

        # Plot related
        self.fig = plt.gcf()
        self.fig.show()
        self.img_array = np.zeros((500,1000))
        self.img = plt.imshow(self.img_array,cmap='gray',vmin=0,vmax=170, interpolation=None)

    def plot_swath(self, ping):
        plt.cla()
        plt.plot(range(-self.side_scan_data.nSamples,0), self.swath_left[::-1])
        plt.plot(range(0,self.side_scan_data.nSamples), self.swath_right)

        plt.xlabel('# of sample per ping') 
        plt.ylabel('Ping return (log compressed)')
        plt.title("Ping %i" % ping)

        plt.pause(10e-5)
        self.fig.canvas.draw()

    def plot_acoustic_image(self, ping):
        self.img_array[ping%500,:500] = np.array(self.swath_left[::-1])
        self.img_array[ping%500,500:] = np.array(self.swath_right)
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
        self.swath_right = [int.from_bytes(byte_val, "big") for byte_val in transducer_raw_right]
        self.side_scan_data.right.append(np.asarray(self.swath_right))

        # Left transducer data handling
        transducer_raw_left = msg.data_one
        self.swath_left = [int.from_bytes(byte_val, "big") for byte_val in transducer_raw_left]
        self.side_scan_data.left.append(np.asarray(self.swath_left))

        ping = len(self.side_scan_data.right)

        self.plot_acoustic_image(ping)
        #self.plot_swath(ping)
        print(ping)

        