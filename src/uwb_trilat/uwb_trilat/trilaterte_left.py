from optparse import OptionParser
import rclpy
from rclpy.node import Node
from scipy.signal import savgol_filter
from scipy.signal import find_peaks
from matplotlib import pyplot as plt
from std_msgs.msg import Float32MultiArray
import numpy as np
import math

plt.ion()

class Trilaterator(Node):
    def __init__(self, topic_name_1, topic_name_2):
        super().__init__('Trilaterated')
        self.s = 180
        self.d = 210
        self.mpp = 5e-3
        self.mph = 6e-3
        self.gamma = (math.pi - math.radians(30))/2
        self.D_1 = []
        self.D_2 = []
        self.samp_size = 920/143
        
        self.ln, = plt.plot([], [], linewidth=0.5, color='b')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic_name_1,
            self.obs_radar_1,
            1
        )
        self.subscription

        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic_name_2,
            self.obs_radar_2,
            1
        )
        self.subscription

        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            'range_bear', 
            1
        )

    def obs_radar_1(self, msg):
        series = msg.data
        series = np.abs(series)

        # apply a Savitzky-Golay filter to the series
        series = savgol_filter(series, 17, 5)

        # find the peaks
        peaks, _ = find_peaks(series, height= self.mph, prominence= self.mpp, width = 10)
        self.D_1 = peaks * self.samp_size
        # self.D_1 = 200 + peaks * self.samp_size
        # print('Distance_1:', self.D_1)

    def obs_radar_2(self, msg):
        series = msg.data
        series = np.abs(series)

        # apply a Savitzky-Golay filter to the series
        series = savgol_filter(series, 17, 5)

        # find the peaks
        peaks, _ = find_peaks(series, height= self.mph, prominence= self.mpp, width = 10)
        self.D_2 = peaks * self.samp_size
        # self.D_2 = 200 + peaks * self.samp_size
        # print('Distance_2:', self.D_2)
    
    def pub_trilaterated(self):
        msg = Float32MultiArray()
        obs = []
        # obs_count = 0
        for l1 in self.D_1:
            for l2 in self.D_2:
                cos_alp = (l1**2 + self.d**2 - l2**2) / (2 * l1 * self.d)
                cos_bet = (l2**2 + self.d**2 - l1**2) / (2 * l2 * self.d)
                if -1 < cos_alp < 1 and -1 < cos_bet < 1:
                    alp = math.acos(cos_alp)
                    bet = math.acos(cos_bet)
                    if alp > self.gamma and bet > self.gamma:
                        rad_range = math.sqrt((l1*cos_alp - self.d/2)**2 + (l1*math.sin(alp) + self.s)**2)
                        if 400 < rad_range < 2500:
                            rad_bear = math.atan2((l1*math.sin(alp) + self.s),(l1*cos_alp - self.d/2))
                            obs.append(rad_range)
                            obs.append(rad_bear)
                            msg.data = obs
                            self.publisher_.publish(msg)
                            # obs_count += 1

            # print('Publishing obs_left', msg.data)
        return 0

def main(args=None):
    parser = OptionParser()
    parser.add_option(
        "-a",
        "--radar1",
        dest="R1",
        help="left radar topic")
    
    parser.add_option(
        "-b",
        "--radar2",
        dest="R2",
        help="left radar topic")
    
    parser.add_option(
        "--ros-args",
        dest="NA1",
        help="ROS2 passes all these commands which will cause errors, so had to mention here")
    
    parser.add_option(
        "-r",
        dest="NA2",
        help="ROS2 passes all these commands which will cause errors, so had to mention here")
    
    (options, _) = parser.parse_args()

    rclpy.init(args=args)
    trilat = Trilaterator(options.R1, options.R2) # Radar topics for left side trilateration
    while rclpy.ok():
        rclpy.spin_once(trilat)
        trilat.pub_trilaterated()

    trilat.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
