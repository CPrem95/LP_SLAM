import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
from std_msgs.msg import ByteMultiArray
from tf_transformations import euler_from_quaternion
import numpy as np
from matplotlib import pyplot as plt
from ekf_slam.ekf_slam.src import ekf_funcs_anchors as eka
from sklearn.cluster import DBSCAN
import pickle
import beepy

plt.ion()

class SLAMSubscriber(Node):
    def __init__(self):
        super().__init__('uwb_SLAM_subscriber')
        
        self.prev_pos = [0,0]
        self.prev_yaw = 0
        self.yaw = 0
        self.trig1 = 1

        self.left_obs_r = []  # range reading from left uwb
        self.left_obs_phi = []  # bearing reading from left uwb
        self.right_obs_r = [] # range reading from right uwb
        self.right_obs_phi = [] # bearing reading from right uwb
        self.len_left_obs = 0
        self.len_right_obs = 0

        self.ini_anc_pos = [0, 0, 0, 0, 0, 0]  # Initialised anchor positions [x1, y1, x2, y2, x3, y3]
        self.anc_reading = [0, 0, 0]  # range reading from anchor 1
        self.trig_initialized = [0, 0, 0]  # whether anchor positions are initialised

        self.count1 = 1

        # Initial experiments 
        self.win1 = 10000
        self.win1_odom = np.zeros([self.win1, 3])
        self.win1_left_obs = np.zeros([self.win1, 5, 2])
        self.win1_right_obs = np.zeros([self.win1, 5, 2])
        self.win1_left_obs_count = np.zeros(self.win1, dtype=int)
        self.win1_right_obs_count = np.zeros(self.win1, dtype=int)

        # Modified experiments
        self.win2 = 10000
        self.win2_odom = np.zeros([self.win1, 3])
        self.win2_left_obs = np.zeros([self.win2*3, 2])
        self.win2_right_obs = np.zeros([self.win2*3, 2])
        self.win2_left_obs_count = np.zeros(self.win2, dtype=int)
        self.win2_right_obs_count = np.zeros(self.win2, dtype=int)
        self.win2_left_obs_prev_tot = 0
        self.win2_left_obs_tot = 0
        self.win2_right_obs_prev_tot = 0
        self.win2_right_obs_tot = 0

        self.win2_left_obs_cart = np.zeros([self.win2*3, 2])
        self.win2_right_obs_cart = np.zeros([self.win2*3, 2])

        # SLAM backend
        n_landm = 20
        ini_landm_var = 1e6
        self.exp_landmarks = n_landm + 50
        self.exp_anchors = 3

        # projection matrix
        self.F = np.block([np.eye(3), np.zeros([3, 2*self.exp_landmarks]), np.zeros([3, 2*self.exp_anchors])])
        
        ini_mu = np.zeros([3 + 2*self.exp_landmarks + 2*self.exp_anchors, 1])

        ini_cov_xx = np.zeros([3, 3])
        ini_cov_xm = np.zeros([3, 2*self.exp_landmarks])
        ini_cov_mx = np.zeros([2*self.exp_landmarks, 3])
        ini_cov_mm = ini_landm_var * np.eye(2*self.exp_landmarks)
        ini_cov_xa = np.zeros([3, 2*self.exp_anchors])
        ini_cov_ax = np.zeros([2*self.exp_anchors, 3])
        ini_cov_ma = np.zeros([2*self.exp_landmarks, 2*self.exp_anchors])
        ini_cov_am = np.zeros([2*self.exp_anchors, 2*self.exp_landmarks])
        ini_cov_aa = np.eye(2*self.exp_anchors)*ini_landm_var
        ini_cov = np.block([[ini_cov_xx, ini_cov_xm, ini_cov_xa],
                        [ini_cov_mx, ini_cov_mm, ini_cov_ma],
                        [ini_cov_ax, ini_cov_am, ini_cov_aa]])
        
        # initial state
        self.sig = ini_cov
        self.mu = ini_mu

        # process noise
        self.R = np.array([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 9e-6]])
        
        # measurement noise
        self.Q = np.array([[22500, 0],
                    [0, 0.6]])
    
        # anchor noise
        self.Q_a = 200**2

        self.fig = plt.figure() 
        self.ax = self.fig.add_subplot(111)
        plt.axis('equal')

        # subscribe to odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            1
        )
        self.subscription

        # subscribe to left uwb range/bearing topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'range_bear_left',
            self.leftObservations_callback,
            1
        )

        # subscribe to right uwb range/bearing topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'range_bear_right',
            self.rightObservations_callback,
            1
        )

        # subscribe to ranges from the Anchors #1
        self.subscription = self.create_subscription(
            Int16,
            'ranges/value1',  # Replace 'anc_1' with the actual topic name
            self.anc_1_callback,
            1  # Adjust the queue size as needed
        )

        # subscribe to ranges from the Anchors #2
        self.subscription = self.create_subscription(
            Int16,
            'ranges/value2',  # Replace 'anc_1' with the actual topic name
            self.anc_2_callback,
            1  # Adjust the queue size as needed
        )

        # subscribe to ranges from the Anchors #3   
        self.subscription = self.create_subscription(
            Int16,
            'ranges/value3',  # Replace 'anc_1' with the actual topic name
            self.anc_3_callback,
            1  # Adjust the queue size as needed
        )

        # subscribe to whether INITIASLISED anchor positions
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'anchor_positions',  # Replace 'anc_1' with the actual topic name
            self.anchor_positions_callback,
            1  # Adjust the queue size as needed
        )

        # publish estimated mu
        self.mu_publisher = self.create_publisher(
            Float32MultiArray, 
            'est_mu',
            1
        )

    # Callback functions
    
    # Callback function for left uwb range/bearing
    def leftObservations_callback(self, msg):
        left_obs = msg.data
        self.left_obs_r = left_obs[::2]
        self.left_obs_phi = left_obs[1::2]
        self.len_left_obs = len(self.left_obs_r)
    
    # Callback function for right uwb range/bearing
    def rightObservations_callback(self, msg):
        right_obs = msg.data
        self.right_obs_r = right_obs[::2]
        self.right_obs_phi = list(np.array(right_obs[1::2]) + np.pi)
        self.len_right_obs = len(self.right_obs_r)

    # Callback functions for range readings from anchors
    def anc_1_callback(self, msg):
        self.anc_reading[0] = int(msg.data)
        # print(self.anc_1_reading)
    def anc_2_callback(self, msg):
        self.anc_reading[1] = int(msg.data)
        # print(self.anc_2_reading)
    def anc_3_callback(self, msg):
        self.anc_reading[2] = int(msg.data)
        # print(self.anc_3_reading)

    def anchor_positions_callback(self, msg):
        self.ini_anc_pos = msg.data

    # def      

    def odometry_callback(self, msg):
        # Your callback function logic here
        # Access odometry data using msg object
        position = msg.pose.pose.position
        odom_x = position.x * 1000
        odom_y = position.y * 1000
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        anc_reading_2 = self.anc_reading
        len_left_obs_2 = self.len_left_obs
        len_right_obs_2 = self.len_right_obs
        left_obs_r_2 = self.left_obs_r
        left_obs_phi_2 = self.left_obs_phi
        right_obs_r_2 = self.right_obs_r
        right_obs_phi_2 = self.right_obs_phi
        
        if (((self.prev_pos[0] - odom_x)**2 + (self.prev_pos[1] - odom_y)**2 > 16) or abs(self.yaw - self.prev_yaw) > 0.001):
            if self.count1 < self.win1:
                print(self.count1)
                self.win2_odom[self.count1, :] = [odom_x, odom_y, self.yaw]
                # print(self.len_left_obs)
                # print(self.left_obs_r)
                # print(self.win1_left_obs[self.count1, 0:self.len_left_obs, 0])
                # print(self.win1_left_obs_count[self.count1])
                # print(self.count1)
                
                ## Initial experiments
                self.win1_left_obs[self.count1, 0:self.len_left_obs, 0] = self.left_obs_r
                self.win1_left_obs[self.count1, 0:self.len_left_obs, 1] = self.left_obs_phi
                self.win1_left_obs_count[self.count1] = self.len_left_obs

                self.win1_right_obs[self.count1, 0:self.len_right_obs, 0] = self.right_obs_r
                self.win1_right_obs[self.count1, 0:self.len_right_obs, 1] = self.right_obs_phi
                self.win1_right_obs_count[self.count1] = self.len_right_obs

                ## Modified experiments
                self.win2_left_obs_tot = self.win2_left_obs_prev_tot + self.len_left_obs
                self.win2_left_obs[self.win2_left_obs_prev_tot:self.win2_left_obs_tot, 0] = self.left_obs_r
                self.win2_left_obs[self.win2_left_obs_prev_tot:self.win2_left_obs_tot, 1] = self.left_obs_phi
                self.win2_left_obs_count[self.count1] = self.len_left_obs
                self.win2_left_obs_prev_tot = self.win2_left_obs_tot

                self.win2_right_obs_tot = self.win2_right_obs_prev_tot + self.len_right_obs
                self.win2_right_obs[self.win2_right_obs_prev_tot:self.win2_right_obs_tot, 0] = self.right_obs_r
                self.win2_right_obs[self.win2_right_obs_prev_tot:self.win2_right_obs_tot, 1] = self.right_obs_phi
                self.win2_right_obs_count[self.count1] = self.len_right_obs

                self.win2_left_obs_cart[self.win2_left_obs_prev_tot:self.win2_left_obs_tot, 0] = self.left_obs_r*np.cos(self.left_obs_phi + self.yaw)
                self.win2_left_obs_cart[self.win2_left_obs_prev_tot:self.win2_left_obs_tot, 1] = self.left_obs_r*np.sin(self.left_obs_phi + self.yaw)

                self.win2_right_obs_cart[self.win2_right_obs_prev_tot:self.win2_right_obs_tot, 0] = self.right_obs_r*np.cos(self.right_obs_phi + self.yaw)
                self.win2_right_obs_cart[self.win2_right_obs_prev_tot:self.win2_right_obs_tot, 1] = self.right_obs_r*np.sin(self.right_obs_phi + self.yaw)
                self.win2_right_obs_prev_tot = self.win2_right_obs_tot

                self.count1 += 1
                # print(self.win1_left_obs_count)
                # print(self.yaw)
                self.prev_pos = [odom_x, odom_y]
                self.prev_yaw = self.yaw
                # a = input('hellooo')
            elif self.count1 == self.win1 and self.trig1:
                with open("win2_odom.pickle", "wb") as f:
                    pickle.dump(self.win2_odom, f)
                with open("win2_left_obs_tot.pickle", "wb") as f:
                    pickle.dump(self.win2_left_obs_tot, f)
                with open("win2_left_obs.pickle", "wb") as f:
                    pickle.dump(self.win2_left_obs, f)
                with open("win2_left_obs_count.pickle", "wb") as f:
                    pickle.dump(self.win2_left_obs_count, f)
                with open("win2_right_obs_tot.pickle", "wb") as f:
                    pickle.dump(self.win2_right_obs_tot, f)
                with open("win2_right_obs.pickle", "wb") as f:
                    pickle.dump(self.win2_right_obs, f)
                with open("win2_right_obs_count.pickle", "wb") as f:
                    pickle.dump(self.win2_right_obs_count, f)
                with open("win2_left_obs_cart.pickle", "wb") as f:
                    pickle.dump(self.win2_left_obs_cart, f)
                with open("win2_right_obs_cart.pickle", "wb") as f:
                    pickle.dump(self.win2_right_obs_cart, f)

                with open("win1_odom.pickle", "wb") as f:
                    pickle.dump(self.win1_odom, f)
                with open("win1_left_obs.pickle", "wb") as f:
                    pickle.dump(self.win1_left_obs, f)
                with open("win1_left_obs_count.pickle", "wb") as f:
                    pickle.dump(self.win1_left_obs_count, f)
                with open("win1_right_obs.pickle", "wb") as f:  
                    pickle.dump(self.win1_right_obs, f)
                with open("win1_right_obs_count.pickle", "wb") as f:
                    pickle.dump(self.win1_right_obs_count, f)

                beepy.beep(sound=1)
                self.trig1 = 0

                if True:    
                    X_left = self.win2_left_obs_cart[0:self.win2_left_obs_tot, :]
                    X_right = self.win2_right_obs_cart[0:self.win2_right_obs_tot, :]
                    clustering_left = DBSCAN(eps=150, min_samples=2).fit(X_left)
                    # clustering.labels array([ 0,  0,  0,  1,  1, -1])
                else:
                    self.ax.plot(self.win1_odom[:, 0], self.win1_odom[:, 1], c='g', label='Path', linewidth=2)
                    self.fig.canvas.draw() 
                    for i in range(self.win1):
                        # print(i)
                        # print(self.win1_left_obs[i, 0:self.win1_left_obs_count[i], 0])
                        # print(self.win1_left_obs[i, 0:self.win1_left_obs_count[i], 1])
                        # print(self.win1_right_obs[i, 0:self.win1_right_obs_count[i], 0])
                        # print(self.win1_right_obs[i, 0:self.win1_right_obs_count[i], 1])
                        # print('*************')
                        # a = input('hellooo')
                        self.ax.scatter(self.win1_odom[i, 0] + self.win1_left_obs[i, 0:self.win1_left_obs_count[i], 0]*np.cos(self.win1_left_obs[i, 0:self.win1_left_obs_count[i], 1] + self.win1_odom[i, 2]), self.win1_odom[i, 1] + self.win1_left_obs[i, 0:self.win1_left_obs_count[i], 0]*np.sin(self.win1_left_obs[i, 0:self.win1_left_obs_count[i], 1] + self.win1_odom[i, 2]), s=10, c='r', marker='o', label='Left Observations')
                        # self.fig.canvas.draw() 
                        self.ax.scatter(self.win1_odom[i, 0] + self.win1_right_obs[i, 0:self.win1_right_obs_count[i], 0]*np.cos(self.win1_right_obs[i, 0:self.win1_right_obs_count[i], 1] + self.win1_odom[i, 2]), self.win1_odom[i, 1] + self.win1_right_obs[i, 0:self.win1_right_obs_count[i], 0]*np.sin(self.win1_right_obs[i, 0:self.win1_right_obs_count[i], 1] + self.win1_odom[i, 2]), s=10, c='b', marker='o', label='Right Observations')
                        # self.fig.canvas.draw() 
                        self.trig1 = 0
                        # self.fig.canvas.draw() 
                        # self.fig.canvas.flush_events()
                        # time.sleep(5)
                    self.fig.canvas.draw() 
                    self.fig.canvas.flush_events()
            else:
                
                u = eka.odom2u(self.win2_odom[0, :], self.win2_odom[1, :])

                mu_bar, sig_bar = eka.ekf_unkown_predict(self.mu, self.sig, u, self.R, self.F)

                obs1 = self.win2_left_obs[0:self.win2_left_obs_count[0],:]
                obs2 = self.win2_right_obs[0:self.win2_right_obs_count[0],:]  
                mu, sig, N = eka.ekf_unknown_correction2(mu_bar, sig_bar, obs1, self.Q, self.exp_landmarks, self.exp_anchors, N, 3)
                mu, sig, N = eka.ekf_unknown_correction2(mu_bar, sig_bar, obs2, self.Q, self.exp_landmarks, self.exp_anchors, N, 3)

                mu, sig, self.trig_initialized = eka.ekf_known_correction_anc2(anc_reading_2, mu, sig, self.exp_landmarks, self.exp_anchors, self.Q_a, self.trig_initialized, self.ini_anc_pos)

                self.mu_publisher.publish(mu)

                # Shift the window
                self.win2_odom[0:self.win2-1, :] = self.win2_odom[1:self.win2, :]   
                # Left observations           
                self.win2_left_obs[0:self.win2_left_obs_tot, :] = self.win2_left_obs[self.win2_left_obs_count[0]:self.win2_left_obs_tot + self.win2_left_obs_count[0], :]
                self.win2_left_obs_cart[0:self.win2_left_obs_tot, :] = self.win2_left_obs_cart[self.win2_left_obs_count[0]:self.win2_left_obs_tot + self.win2_left_obs_count[0], :]
                self.win2_left_obs_tot = self.win2_left_obs_tot + len_left_obs_2 - self.win2_left_obs_count[0]
                
                # Right observations
                self.win2_right_obs[0:self.win2_right_obs_tot, :] = self.win2_right_obs[self.win2_right_obs_count[0]:self.win2_right_obs_tot + self.win2_right_obs_count[0], :]
                self.win2_right_obs_cart[0:self.win2_right_obs_tot, :] = self.win2_right_obs_cart[self.win2_right_obs_count[0]:self.win2_right_obs_tot + self.win2_right_obs_count[0], :]
                self.win2_right_obs_tot = self.win2_right_obs_tot + len_right_obs_2 - self.win2_right_obs_count[0]
                
                # Add new observations
                self.win2_odom[-1, :] = [odom_x, odom_y, self.yaw]
                self.win2_left_obs_count[-1] = len_left_obs_2
                self.win2_right_obs_count[-1] = len_right_obs_2
                self.win2_left_obs[self.win2_left_obs_prev_tot - self.win2_left_obs_count[0]:self.win2_left_obs_tot, 0] = left_obs_r_2
                self.win2_left_obs[self.win2_left_obs_prev_tot - self.win2_left_obs_count[0]:self.win2_left_obs_tot, 1] = left_obs_phi_2
                self.win2_left_obs_cart[self.win2_left_obs_prev_tot - self.win2_left_obs_count[0]:self.win2_left_obs_tot, 0] = left_obs_r_2*np.cos(left_obs_phi_2 + self.yaw)
                self.win2_left_obs_cart[self.win2_left_obs_prev_tot - self.win2_left_obs_count[0]:self.win2_left_obs_tot, 1] = left_obs_r_2*np.sin(left_obs_phi_2 + self.yaw)

                self.win2_right_obs[self.win2_right_obs_prev_tot - self.win2_right_obs_count[0]:self.win2_right_obs_tot, 0] = right_obs_r_2
                self.win2_right_obs[self.win2_right_obs_prev_tot - self.win2_right_obs_count[0]:self.win2_right_obs_tot, 1] = right_obs_phi_2
                self.win2_right_obs_cart[self.win2_right_obs_prev_tot - self.win2_right_obs_count[0]:self.win2_right_obs_tot, 0] = right_obs_r_2*np.cos(right_obs_phi_2 + self.yaw)
                self.win2_right_obs_cart[self.win2_right_obs_prev_tot - self.win2_right_obs_count[0]:self.win2_right_obs_tot, 1] = right_obs_r_2*np.sin(right_obs_phi_2 + self.yaw)

                self.win2_left_obs_prev_tot = self.win2_left_obs_tot
                self.win2_right_obs_prev_tot = self.win2_right_obs_tot

                self.prev_pos = [odom_x, odom_y]
                self.prev_yaw = self.yaw
            

def main(args=None):
    rclpy.init(args=args)
    slam = SLAMSubscriber()
    while rclpy.ok():
        rclpy.spin(slam)

    slam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

