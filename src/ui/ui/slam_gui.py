# -- coding: utf-8 --
"""
Created on Tue Mar 26 14:14:23 2024

@author: PhD-HanchapolaAppuha
"""

#!/usr/bin/env python

import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

N_lms = 3 # Expected number of landmarks
range_N_lms = 2*N_lms + 3
N_ancs = 2 # Expected number of anchors
range_N_ancs = range_N_lms + 2*N_ancs

class SLAMSubscriber(Node):
    def __init__(self):
        super().__init__('uwb_SLAM_visualizer')

        self.x_data = []
        self.y_data = []
        self.lm_data_x = []
        self.lm_data_y = []
        self.anc_data_x = []
        self.anc_data_y = []

        self.prev_odom_x = 0
        self.prev_odom_y = 0
        self.prev_odom_th = 0
        
        self.subscription = self.create_subscription(
            Float32MultiArray, 
            'est_mu',
            self.visual_callback,
            10) # 10 Hz
        self.subscription

    def visual_callback(self, msg):
        mu = msg.data
        if mu[0] != self.prev_odom_x or mu[1] != self.prev_odom_y or mu[2] != self.prev_odom_th:
            self.x_data.append(mu[0])
            self.y_data.append(mu[1])
        
        self.lm_data_x = mu[3:range_N_lms:2]
        self.lm_data_y = mu[4:range_N_lms:2]

        self.anc_data_x = mu[range_N_lms:range_N_ancs:2]
        self.anc_data_y = mu[range_N_lms +1:range_N_ancs:2]

        plt.clf()
        plt.grid(color='gray', linestyle='--', linewidth=0.5)
        
        plt.plot(self.x_data, self.y_data, c='b', label='Estimated Path')
        plt.scatter(self.lm_data_x, self.lm_data_y, c='m', marker='o', label='Estimated Landmarks')
        plt.scatter(self.anc_data_x, self.anc_data_y, c='r', marker='^', label='Estimated Anchors')
        plt.xlabel('X [mm]')
        plt.ylabel('Y [mm]')
        plt.title('UWB SLAM Visualization')
        plt.rcParams['axes.axisbelow'] = True
        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')
        plt.legend(loc='best')
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    slam_subscriber = SLAMSubscriber()
    rclpy.spin(slam_subscriber)
    slam_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()