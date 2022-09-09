import matplotlib.pyplot as plt
import numpy as np

class plot_utils:
    def __init__(self):
        self.fig, self.axs = plt.subplots(1)
        self.fig.show()
        #self.img_array = np.zeros((6000,6000))
        #self.img = plt.imshow(self.img_array,cmap='gray',vmin=0,vmax=2, interpolation=None)
        #self.img_raw_array = np.zeros((500,1000))
        plt.ion()


    def plot_swath(self, ping, altitude, side_scan_data, swath_right, spline_right, swath_left=None, spline_left=None):
        self.axs.cla()
        if swath_left is None:
            # Plotting raw swath and spline fitted curve
            self.axs.plot(range(0,side_scan_data.nSamples), swath_right, color='blue')
            self.axs.plot(range(0,side_scan_data.nSamples), spline_right, color='orange', linestyle='dashed')
            self.axs.legend(["Across-track signal","Cubic spline fitted curve"], loc="upper right")
        else:
            self.axs.plot(range(-side_scan_data.nSamples,0), swath_left[::-1], color='blue')
            self.axs.plot(range(-side_scan_data.nSamples,0), spline_left[::-1],
                          range(0,side_scan_data.nSamples),  spline_right, color='orange', zorder=3)
            self.axs.plot(range(0,side_scan_data.nSamples),  swath_right, color='blue')
            self.axs.axvline(x=0, ymin=0, color='black', linestyle='dotted')

            self.axs.legend(["Across-track signal", "Cubic spline fitted curve"], loc="upper right")
            
        plt.axvline(x=np.floor_divide(altitude, side_scan_data.res), color='black', linestyle='--')
        self.axs.set(xlabel='# of sample per ping', ylabel='Ping return (log compressed)')
        self.axs.set_title("Ping %i" % ping)
        plt.gca().axis('tight')

        plt.pause(10e-5)
        self.fig.canvas.draw()

    def plot_CV(self, ping, raw_swath, normalized_swath, CV_raw, CV_normalized):
        #plt.cla()
        CV_raw.append(np.std(raw_swath) / np.mean(raw_swath))
        CV_normalized.append(np.std(normalized_swath) / np.mean(normalized_swath))

        self.axs.plot(range(0,ping), CV_raw, range(0,ping), CV_normalized)
        self.axs.legend(["Raw swath", "Normalized swath"], loc="upper right")

        self.axs.set_title("Across-track Coefficient of Variation")

        plt.gca().axis('tight')
        plt.pause(10e-5)
        self.fig.canvas.draw()

    def plot_acoustic_image(self):#, ping, swath_right, swath_left):
        #self.img_array[ping%500,:500] = np.array(swath_left[::-1])
        #self.img_array[ping%500,500:] = np.array(swath_right)
        #self.img.set_data(self.img_array)
        
        plt.xlabel('Across track') 
        plt.ylabel('Along track')
        plt.title("Acoustic Image")

        # Limiting refresh rate due to time consumption
        #if ping%400 == 0:
        #    plt.pause(10e-5)
        #    self.fig.canvas.draw()
        plt.pause(10e-5)
        self.fig.canvas.draw()

    #def plot_raw_image(self, fig, axis, raw_swath_array):
    #    axis.imshow(raw_swath_array,cmap='gray', interpolation=None)
    #    axis.set(xlabel='Across track', ylabel='Along track', title='Raw Swath Frame')
    #    plt.pause(10e-5)
    #    fig.canvas.draw()

    def plot_raw_image(self, fig, axis, u, v, intensity_values):
        axis.scatter(v,u,c=intensity_values,s=0.01,cmap='gray')
        axis.set(xlabel='Across track', ylabel='Along track', title='Raw Swath Frame')
        plt.pause(10e-5)
        fig.canvas.draw()

    def plot_global_batch_image(self, fig, axis, processed_swath_array):
        axis.imshow(processed_swath_array,cmap='gray', interpolation=None)
        axis.set(xlabel='Across track', ylabel='Along track', title='Processed Swath Frame')
        plt.pause(10e-5)
        fig.canvas.draw()

    def plot_global_image(self, fig, axis, global_processed_swath_array):
        axis.imshow(global_processed_swath_array,cmap='gray', interpolation=None)
        axis.set(xlabel='Across track', ylabel='Along track', title='Global Processed Frame')
        plt.pause(10e-5)
        fig.canvas.draw()


    def plot_local_image():
        pass

    def plot_image_construction():
        pass