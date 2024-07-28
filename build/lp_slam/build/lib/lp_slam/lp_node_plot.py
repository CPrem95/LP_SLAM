# This program was created to subscribe to mu and plot the estimated landmarks and the robot path.
# It is still in development.**********************************************************************************************!!!!!!!!

from lp_slam.src import ekf_funcs_lp as ekf
import matplotlib
from matplotlib import pyplot as plt
import time
import numpy as np
import pickle
import math
import pdb
import math
import scipy.io

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as rot

import threading

np.random.seed(10) # for reproducibility
plt.ion()

class USLAM(Node):
    def __init__(self, topic_name_odom, topic_name_tri_L, topic_name_tri_R, topic_name_mu):
        super().__init__('USLAM')
        ############################################################################################################
        # VARIABLES
        ############################################################################################################
        # **********************************************************************************************
        # USER-DEFINED VARS
        self.winsize = 100 # window size for a region
        ## Define the window size for BOTH REGIONS R12
        self.winsize2 = int(1.5*self.winsize) # 1.5 times the window size for both regions
        self.stepsize = 3 # step size for the SLAM update window
        self.max_steps = 30000 # maximum number of steps >> self.odom, self.mu_hist

        ini_pt_landm_var = 1e6 # initial point landmark variance
        ini_ln_landm_var = 1e4 # initial line landmark variance
        self.exp_pt_landm = 50 # expected number of point landmarks
        self.exp_line_landm = 50 # expected number of line landmarks

        # process noise
        self.R = np.array([[100, 0, 0],
                    [0, 100, 0],
                    [0, 0, 0.0004]])
        
        # measurement noise points
        self.Q_pts = np.array([[10000, 0],
                    [0, 0.25]])
        
        # measurement noise lines
        self.Q_lines = np.array([[2500, 0],
                    [0, 0.25]])
        
        # (1,2) Regions
        N_inl_thresh_12 = 60
        L_len_thresh_12 = 750
        r_thresh_12 = 300
        th_thresh_12 = math.radians(10)
        self.thresh_12 = [N_inl_thresh_12, L_len_thresh_12, r_thresh_12, th_thresh_12]

        # (1) Regions   
        N_inl_thresh_1 = 40 # 30
        L_len_thresh_1 = 500
        r_thresh_1 = 100
        th_thresh_1 = math.radians(10)
        self.thresh_1 = [N_inl_thresh_1, L_len_thresh_1, r_thresh_1, th_thresh_1]

        # (2) Regions
        N_inl_thresh_2 = 30
        L_len_thresh_2 = 350
        r_thresh_2 = r_thresh_1
        th_thresh_2 = th_thresh_1
        thresh_2 = [N_inl_thresh_2, L_len_thresh_2, r_thresh_2, th_thresh_2]

        # **********************************************************************************************
        # ROS VARS
        self.del_l = 0
        self.del_th = 0
        self.prev_odom = np.zeros(3, dtype=np.float32)
        self.id = 1 # id of the step = 1 because the first step is used to get the initial observations
        self.prev_id = self.id # previous id
        self.pose_id = 0 # id of the pose

        # initialise vars for trilateration callbacks 
        self.tmp_l_obs = [] # temporary left observations >> range and bearing
        self.tmp_l_obs_xy = [] # temporary left observations >> x and y
        self.tmp_l_obs_count = 0 # temporary left observations count
        self.tmp_r_obs = [] # temporary right observations >> range and bearing
        self.tmp_r_obs_xy = [] # temporary right observations >> x and y
        self.tmp_r_obs_count = 0 # temporary right observations count

        # odometry data
        self.odom = np.zeros([self.max_steps, 3], dtype=np.float32)

        # observation vars 
        # Find a way to avoid zero initialization
        self.l_obs_data = [[0, 0]]
        self.l_obs_xy = [[0, 0]]
        self.count_l_obs = [0]

        self.r_obs_data = [[0, 0]]
        self.r_obs_xy = [[0, 0]]
        self.count_r_obs = [0]
        
        # **********************************************************************************************
        # EKF-SLAM VARIABLES

        # projection matrix
        self.F = np.block([np.eye(3), np.zeros([3, 2*self.exp_pt_landm + 2*self.exp_line_landm])])

        # initial state
        self.mu = np.zeros([3 + 2*self.exp_pt_landm + 2*self.exp_line_landm, 1])

        # initial covariance matrix
        ini_cov_xx = np.zeros([3, 3])
        ini_cov_xp = np.zeros([3, 2*self.exp_pt_landm])
        ini_cov_px = np.zeros([2*self.exp_pt_landm, 3])
        ini_cov_pp = ini_pt_landm_var * np.eye(2*self.exp_pt_landm)
        ini_cov_xl = np.zeros([3, 2*self.exp_line_landm])
        ini_cov_lx = np.zeros([2*self.exp_line_landm, 3])
        ini_cov_ml = np.zeros([2*self.exp_pt_landm, 2*self.exp_line_landm])
        ini_cov_lm = np.zeros([2*self.exp_line_landm, 2*self.exp_pt_landm])
        ini_cov_ll = np.eye(2*self.exp_line_landm)*ini_ln_landm_var

        self.sig = np.block([[ini_cov_xx, ini_cov_xp, ini_cov_xl],
                            [ini_cov_px, ini_cov_pp, ini_cov_ml],
                            [ini_cov_lx, ini_cov_lm, ini_cov_ll]])
        
        self.N_pts = 0 # no observed points in the beginning
        self.N_line = 0 # no observed lines in the beginning

        self.visLine_x = np.zeros([self.exp_line_landm, 2])
        self.visLine_y = np.zeros([self.exp_line_landm, 2])

        # **********************************************************************************************
        # PLOTTING VARS
        self.raw_plot = False
        self.slam_plot = True

        # Figure for raw observations plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111) 
        self.ax.set_aspect('equal', adjustable='box')
        # plt.axis('equal')
        plt.grid(linestyle="--", color='black', alpha=0.3)
        self.ax.set_xlabel('x [mm]')
        self.ax.set_ylabel('y [mm]')
        self.ax.set_title('Odometry-based observations')
        plt.rcParams['backend'] = 'Qt5Agg'
        plt.show()

        # Figure for final estimated lines and points plot
        self.fig2 = plt.figure()
        self.ax2 = self.fig2.add_subplot(111)
        self.ax2.set_aspect('equal', adjustable='box')
        plt.grid(linestyle="--", color='black', alpha=0.3)
        self.ax2.set_xlabel('x [mm]')
        self.ax2.set_ylabel('y [mm]')
        self.ax2.set_title('Final SLAM output')
        plt.rcParams['backend'] = 'Qt5Agg'
        plt.show()

        # Globally filtered observations
        # plt_lines_all, = ax.plot([], [], c='purple', linewidth = 2, label='Final estimated lines')
        self.plt_lines_all = []
        for i in range(10):
            tmp_plt_ln_all, = self.ax.plot([], [], c='red', linewidth = 2, linestyle= '--', label='Final est lines')
            self.plt_lines_all.append(tmp_plt_ln_all)
            self.fig.canvas.draw()
        self.plt_points_all = self.ax.scatter([], [], s = 30, edgecolors='k', facecolors= 'red', marker= 'o', label='Final est points')
        self.fig.canvas.draw()

        self.mu_hist_x = [0] # history of the robot pose
        self.mu_hist_y = [0] # history of the robot pose
        
        # self.obs_count = np.zeros([1, self.exp_pt_landm + self.exp_line_landm])
        # self.hist_i = np.zeros([1, self.exp_pt_landm + self.exp_line_landm])
        self.obs_count = 0
        self.hist_i = 0

        # **********************************************************************************************
        # SLAM Plot
        # Estimated path plot
        self.raw_odom, = self.ax2.plot([], [], linewidth = 2, c='k', label='Odometry')
        self.fig2.canvas.draw()
        self.est_path, = self.ax2.plot([], [], linewidth = 2, c='b', label='Est path')
        self.fig2.canvas.draw()
        # Estimated point landmarks plot
        self.est_pt_lms = self.ax2.scatter([], [], s=30, c='red', marker='o', label='Est Pt LMs')
        self.fig2.canvas.draw()
        # Estimated line landmarks plot
        self.est_ln_lms = []
        for i in range(self.exp_line_landm):
            tmp_plt_ln_all, = self.ax2.plot([], [], c='red', linewidth = 2, linestyle= '-', label='Est Ln LMs')
            self.est_ln_lms.append(tmp_plt_ln_all)
            self.fig.canvas.draw()

        # Legend
        self.fig2.legend(handles=[self.raw_odom, self.est_path, self.est_pt_lms, tmp_plt_ln_all], loc='upper right')
        self.fig2.canvas.flush_events()
    

        ############################################################################################################
        # SUBSCRIPTIONS
        ############################################################################################################
        # Save data from the odometry
        self.subscription = self.create_subscription(
            Odometry,
            topic_name_odom,
            self.update_odom,
            1
        )
        self.subscription

        # Save left trilateration data
        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic_name_tri_L,
            self.update_l_obs,
            1
        )
        self.subscription

        # Save right trilateration data
        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic_name_tri_R,
            self.update_r_obs,
            1
        )
        self.subscription

        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            topic_name_mu, 
            10
        )

    ############################################################################################################
    # CALLBACK FUNCTIONS
    ############################################################################################################
    # Update the left observations
    def update_l_obs(self, msg):
        obs = [element for element in msg.data if element != 0]
        L = len(obs)
        tmp_obs = []
        tmp_obs_xy = []
        for i in range(0, L, 2):
            tmp_obs.append([obs[i], obs[i+1]])
            tmp_obs_xy.append(rth2xy(self.prev_odom, obs[i], obs[i+1]))  # the prev_odom is updated to the current odom pose in the previous step
        self.tmp_l_obs = tmp_obs
        self.tmp_l_obs_xy = tmp_obs_xy
        # print('tmp_l_obs_xy: ', self.tmp_l_obs_xy)
        self.tmp_l_obs_count = L//2
        return 0
    # Update the the right observations
    def update_r_obs(self, msg):
        obs = [element for element in msg.data if element != 0]
        L = len(obs)
        tmp_obs = []
        tmp_obs_xy = []
        for i in range(0, L, 2):
            tmp_obs.append([obs[i], obs[i+1]])
            tmp_obs_xy.append(rth2xy(self.prev_odom, obs[i], obs[i+1]))  # the prev_odom is updated to the current odom pose in the previous step
        self.tmp_r_obs = tmp_obs
        self.tmp_r_obs_xy = tmp_obs_xy
        # print('tmp_r_obs_xy: ', self.tmp_r_obs_xy)
        self.tmp_r_obs_count = L//2
        return 0
    
    # Update the raw observations
    def update_obs(self):
        # self.l_obs_data = np.concatenate((self.l_obs_data, self.tmp_l_obs), axis=0) # concatenate lists
        self.l_obs_data = self.l_obs_data + self.tmp_l_obs # concatenate lists
        self.count_l_obs.append(self.tmp_l_obs_count)
        # self.l_obs_xy = np.concatenate((self.l_obs_xy, self.tmp_l_obs_xy), axis=0) # concatenate lists
        self.l_obs_xy = self.l_obs_xy + self.tmp_l_obs_xy # concatenate lists

        # self.r_obs_data = np.concatenate((self.r_obs_data, self.tmp_r_obs), axis=0) # concatenate lists
        self.r_obs_data = self.r_obs_data + self.tmp_r_obs # concatenate lists
        self.count_r_obs.append(self.tmp_r_obs_count)
        # self.r_obs_xy = np.concatenate((self.r_obs_xy, self.tmp_r_obs_xy), axis=0) # concatenate lists
        self.r_obs_xy = self.r_obs_xy + self.tmp_r_obs_xy # concatenate lists
        return 0
    
    def get_init_obs(self, init_n):
        return 0
    
    def slam(self):
        return 0
    
    def pub_mu(self, mu):
        msg = Float32MultiArray()
        msg.data = mu
        self.publisher_.publish(msg)
            # print('Publishing obs_left', msg.data)
        return 0
    
    def update_plot_obs(self):
        # Update the plot
        # while True:
        if self.slam_plot and self.prev_id != self.id and self.id > self.winsize2:
            # Plot the raw odometry
            self.raw_odom.set_xdata(self.odom[0:self.id, 0])
            self.raw_odom.set_ydata(self.odom[0:self.id, 1])
            # Plot the estimated path
            self.est_path.set_xdata(self.mu_hist_x)
            self.est_path.set_ydata(self.mu_hist_y)
            # Plot the estimated landmarks
            # print("mu:", mu[3:3+2*N_pts].reshape(-1, 2))
            # print("mu:", mu)
            self.est_pt_lms.set_offsets(self.mu[3:3+2*self.N_pts].reshape(-1, 2))
            # for i in range(N_line):
            #     est_ln_lms[i].set_xdata([mu[3+2*exp_pt_landm + 2*i][0], mu[3+2*exp_pt_landm + 2*i+1][0]])
            #     est_ln_lms[i].set_ydata([mu[3+2*exp_pt_landm + 2*i][1], mu[3+2*exp_pt_landm + 2*i+1][1]])

            # Adjust the axis limits
            self.ax2.set_xlim(min(self.odom[:, 0]) - 2500, max(self.odom[:, 0]) + 2500)
            self.ax2.set_ylim(min(self.odom[:, 1]) - 2500, max(self.odom[:, 1]) + 2500)

            self.fig2.canvas.draw()

            # Plot the estimated landmarks
            # Lines
            lnstart = self.exp_pt_landm*2 + 3
            pi = math.pi
            for lin_i in range (self.N_line):
                
                line_r = self.mu[2*lin_i + lnstart]
                line_th = self.mu[2*lin_i + lnstart + 1]

                line_m = math.tan(pi/2 + line_th)
                line_c = line_r/math.sin(line_th)

                if abs(abs(line_th) - pi/2) < pi/4:
                    x = self.visLine_x[lin_i, :]
                    y = line_m*x + line_c
                else:
                    y = self.visLine_y[lin_i, :]
                    x = (y - line_c)/line_m

                self.est_ln_lms[lin_i].set_xdata(x)
                self.est_ln_lms[lin_i].set_ydata(y)
                self.fig.canvas.draw()
            
            # Points
            self.est_pt_lms.set_offsets(self.mu[3:3+2*self.N_pts].reshape(-1, 2))
            self.fig2.canvas.draw()

            self.fig2.canvas.flush_events()


    # Callback functions odometry data
    def update_odom(self, msg):
        prev_x = self.prev_odom[0]
        prev_y = self.prev_odom[1]
        prev_theta = self.prev_odom[2]

        cur_x = msg.pose.pose.position.x * 1000
        cur_y = msg.pose.pose.position.y * 1000
        cur_theta = rot.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]).as_euler('zyx')[0]

        del_x = cur_x - prev_x
        del_y = cur_y - prev_y
        self.del_l = del_x**2 + del_y**2
        self.del_th = abs(cur_theta - prev_theta)
        
        if self.del_l >= 20 or self.del_th >= 2e-3:
            self.prev_odom = [cur_x, cur_y, cur_theta]
            self.odom[self.id] = [cur_x, cur_y, cur_theta]
            print('id:', self.id)
            self.update_obs()

            if self.id < self.winsize2:
                pass
            
            else:
                if self.pose_id%self.stepsize == 0:
                    print('pose_id:', self.pose_id)
                    # **********************************************************************************************
                    # Preparing the observations for the EKF-SLAM

                    # Find indices for the observations in the current window || REGION 1
                    start_ind_lhs_R1 = sum(self.count_l_obs[0:self.pose_id])
                    end_ind_lhs_R1 = sum(self.count_l_obs[0:self.pose_id+self.winsize])
                    start_ind_rhs_R1 = sum(self.count_r_obs[0:self.pose_id])
                    end_ind_rhs_R1 = sum(self.count_r_obs[0:self.pose_id+self.winsize])

                    # Slice the observations in the current window using the indices above || REGION 1
                    obs_lhs_R1 = self.l_obs_xy[start_ind_lhs_R1:end_ind_lhs_R1][:]
                    obs_rhs_R1 = self.r_obs_xy[start_ind_rhs_R1:end_ind_rhs_R1][:]

                    # Find indices for the observations in the current window || REGION 12
                    start_ind_lhs_R12 = sum(self.count_l_obs[0:self.pose_id])
                    end_ind_lhs_R12 = sum(self.count_l_obs[0:self.pose_id+self.winsize2])
                    start_ind_rhs_R12 = sum(self.count_r_obs[0:self.pose_id])
                    end_ind_rhs_R12 = sum(self.count_r_obs[0:self.pose_id+self.winsize2])

                    # Slice the observations in the current window using the indices above || REGION 12
                    obs_lhs_R12 = self.l_obs_xy[start_ind_lhs_R12:end_ind_lhs_R12][:]
                    obs_rhs_R12 = self.r_obs_xy[start_ind_rhs_R12:end_ind_rhs_R12][:]

                    ############################################################################################################
                    # EKF-SLAM update
                    ############################################################################################################
                    odom0 = self.odom[self.pose_id]
                    odom1 = self.odom[self.pose_id+self.stepsize]

                    # control input
                    u = ekf.odom2u(odom0, odom1)
                    # prediction
                    self.mu_bar, self.sig_bar = ekf.ekf_unkown_predict(self.mu, self.sig, u, self.R, self.F)
                    # Update the observations based on the predicted odometry
                    del_odom = odom0 - self.mu[0:3].reshape(1, -1)[0] # difference between the predicted and the actual odometry

                    obs_lhs_R1 = obs_lhs_R1 - odom1[:2]
                    obs_rhs_R1 = obs_rhs_R1 - odom1[:2]
                    obs_lhs_R12 = obs_lhs_R12 - odom1[:2]
                    obs_rhs_R12 = obs_rhs_R12 - odom1[:2]

                    # Rotate the observations to the robot frame
                    res_mu_bar = self.mu_bar[0:2].reshape(1, -1)[0]

                    obs_lhs_R1 = ekf.rotate_points(obs_lhs_R1, del_odom[2]) + res_mu_bar
                    obs_rhs_R1 = ekf.rotate_points(obs_rhs_R1, del_odom[2]) + res_mu_bar
                    obs_lhs_R12 = ekf.rotate_points(obs_lhs_R12, del_odom[2]) + res_mu_bar
                    obs_rhs_R12 = ekf.rotate_points(obs_rhs_R12, del_odom[2]) + res_mu_bar

                    ############################################################################################################
                    ## Observation Extraction
                    ############################################################################################################
                    # rob_obs_pose = mu_bar[0:3]
                    rob_obs_pose = self.mu_bar[0:3].reshape(-1, 1)
                    minNNI_12 = 50

                    odom_R12 = self.odom[self.pose_id:self.pose_id+self.winsize2, :]
                    odom_R1 = self.odom[self.pose_id:self.pose_id+self.winsize, :]

                    # REGION 12 line and point visualization
                    # REGION 1 line and point visualization
                    # FINAL All line and point visualization
                    vis_PL12, vis_PL1, vis_PL_all = True, True, True
                    if not self.raw_plot: 
                        vis_PL12, vis_PL1, vis_PL_all = False, False, False

                    # LHS observations REGION 12
                    vis = False # visualize regressed lines
                    # print('Region 12 LHS observations:')
                    d_thresh = 300
                    N_LMs_lhs_12 = np.zeros([2], dtype=int)
                    N_LMs_lhs_12, L_LMs_lhs_12 = ekf.lineExtract(obs_lhs_R12, odom_R12, rob_obs_pose, self.thresh_12, N_LMs_lhs_12, minNNI_12, vis) # Extract lines from the observations
                    params = [15, 200] # [min_pts, max_rad]
                    N_LMs_lhs_12, P_LMs_lhs_12, P_LMs_lhs_12_plt = ekf.pointExtract(obs_lhs_R12, L_LMs_lhs_12, N_LMs_lhs_12, d_thresh, params, odom_R12, rob_obs_pose, vis) # Extract points from the observations:: (points, L_LMs, N_LMs, d_thresh, params, odom_i, rob_obs_pose, fig):
                    # if vis_PL12:
                    #     ekf.visPLs2(N_LMs_lhs_12, L_LMs_lhs_12, P_LMs_lhs_12_plt, i, plt_lines_lhs_R12, plt_points_lhs_R12, fig) # visualize on top of the odom based obs plot

                    # RHS observations REGION 12
                    vis = False # visualize regressed lines
                    # print('Region 12 RHS observations:')
                    d_thresh = 300
                    N_LMs_rhs_12 = np.zeros([2], dtype=int)
                    N_LMs_rhs_12, L_LMs_rhs_12 = ekf.lineExtract(obs_rhs_R12, odom_R12, rob_obs_pose, self.thresh_12, N_LMs_rhs_12, minNNI_12, vis) # Extract lines from the observations
                    params = [15, 200] # [min_pts, max_rad]
                    N_LMs_rhs_12, P_LMs_rhs_12, P_LMs_rhs_12_plt = ekf.pointExtract(obs_rhs_R12, L_LMs_rhs_12, N_LMs_rhs_12, d_thresh, params, odom_R12, rob_obs_pose, vis) # Extract points from the observations:: (points, L_LMs, N_LMs, d_thresh, params, odom_i, rob_obs_pose, fig):
                    # if vis_PL12:
                    #     ekf.visPLs2(N_LMs_rhs_12, L_LMs_rhs_12, P_LMs_rhs_12_plt, i, plt_lines_rhs_R12, plt_points_rhs_R12, fig) # visualize on top of the odom based obs plot

                    # LHS observations REGION 1
                    vis = False
                    # print('Region 1 LHS observations:')
                    d_thresh = 100
                    N_LMs_lhs_1 = np.zeros([2], dtype=int)
                    N_LMs_lhs_1, L_LMs_lhs_1 = ekf.lineExtract(obs_lhs_R1, odom_R1, rob_obs_pose, self.thresh_1, N_LMs_lhs_1, minNNI_12, vis) # Extract lines from the observations
                    params = [10, 200] # [min_pts, max_rad]
                    N_LMs_lhs_1, P_LMs_lhs_1, P_LMs_lhs_1_plt = ekf.pointExtract(obs_lhs_R1, L_LMs_lhs_1, N_LMs_lhs_1, d_thresh, params, odom_R1, rob_obs_pose, vis)
                    # if vis_PL1:
                    #     ekf.visPLs2(N_LMs_lhs_1, L_LMs_lhs_1, P_LMs_lhs_1_plt, i, plt_lines_lhs_R1, plt_points_lhs_R1, fig)

                    # RHS observations REGION 1
                    vis = False
                    # print('Region 1 RHS observations:')
                    d_thresh = 100
                    N_LMs_rhs_1 = np.zeros([2], dtype=int)
                    N_LMs_rhs_1, L_LMs_rhs_1 = ekf.lineExtract(obs_rhs_R1, odom_R1, rob_obs_pose, self.thresh_1, N_LMs_rhs_1, minNNI_12, vis) # Extract lines from the observations
                    params = [10, 200] # [min_pts, max_rad]
                    N_LMs_rhs_1, P_LMs_rhs_1, P_LMs_rhs_1_plt = ekf.pointExtract(obs_rhs_R1, L_LMs_rhs_1, N_LMs_rhs_1, d_thresh, params, odom_R1, rob_obs_pose, vis)
                    # if vis_PL1:
                    #     ekf.visPLs2(N_LMs_rhs_1, L_LMs_rhs_1, P_LMs_rhs_1_plt, i, plt_lines_rhs_R1, plt_points_rhs_R1, fig)

                    # Filter observations::
                    obs_Lin, obs_Pt = ekf.createObs(N_LMs_lhs_1, L_LMs_lhs_1, P_LMs_lhs_1, N_LMs_lhs_12, L_LMs_lhs_12, 
                                                    N_LMs_rhs_1, L_LMs_rhs_1, P_LMs_rhs_1, N_LMs_rhs_12, L_LMs_rhs_12, 
                                                    rob_obs_pose, self.mu, self.mu_bar, self.N_line, self.exp_pt_landm, self.visLine_x, self.visLine_y, vis_PL_all, self.plt_lines_all, self.plt_points_all) 

                    ############################################################################################################
                    # EKF-SLAM update
                    ############################################################################################################
                    self.mu, self.sig, self.N_pts, self.N_line, _, _, self.visLine_x, self.visLine_y = ekf.EKF_unknown_correction_LP(self.mu_bar, self.sig_bar, rob_obs_pose, obs_lhs_R12, obs_rhs_R12, obs_Pt, obs_Lin, self.Q_pts, self.Q_lines, self.pose_id, self.hist_i, self.obs_count, self.exp_pt_landm, self.exp_line_landm, self.N_pts, self.N_line, self.visLine_x, self.visLine_y, 0.1, 2.5) # alp_pt, alp_line
                    print("\nObserved LMs: ", self.N_pts, self.N_line)
                    
                    self.mu_hist_x.append(self.mu[0]) # save the history of the robot pose
                    self.mu_hist_y.append(self.mu[1]) 
                    self.pub_mu(self.mu) # publish the estimated mu

                    # Update the plot
                    self.update_plot_obs()

                self.pose_id += 1   
            self.prev_id = self.id
            self.id += 1



    
                

                
############################################################################################################
## User-defined FUNCTIONS 
############################################################################################################
def set_limits(ax, X, odom = []):
    # odom = np.array(odom).reshape(-1, 2)
    # print("odom:", odom)
    # print("X:", X)
    # Sets the axis limits to include all the data points in X
    data = np.concatenate((X, odom[:, 0:2]), axis=0)
    xmin=min(data[:, 0]); xmax=max(data[:, 0])
    ymin=min(data[:, 1]); ymax=max(data[:, 1])
    ax.set_xlim(xmin-0.1*(xmax-xmin) - 1000, xmax+0.1*(xmax-xmin) + 1000)
    ax.set_ylim(ymin-0.1*(ymax-ymin) - 1000, ymax+0.1*(ymax-ymin) + 1000)

def set_odom_dir(fig, odom_dir, odom1):
    # Set the direction of the odometry
    odom_dir.set_xdata([odom1[0], odom1[0] + 200*math.cos(odom1[2])])
    odom_dir.set_ydata([odom1[1], odom1[1] + 200*math.sin(odom1[2])])
    fig.canvas.draw()
    fig.canvas.flush_events()

def rth2xy(pose, r, th):
        x = pose[0] + r * math.cos(th + pose[2])
        y = pose[1] + r * math.sin(th + pose[2])
        return [x, y]

def main():
    rclpy.init()
    uwb_slam = USLAM('odom', 'LeftObs/range_bear', 'RightObs/range_bear', 'estimated_mu')
    def spin_thread():
        rclpy.spin(uwb_slam)

    # # Create a new thread
    # ros_thread = threading.Thread(target=spin_thread)
    # ros_thread.start()

    # # Create a new thread
    # plot_thread = threading.Thread(target=uwb_slam.update_plot_obs)
    # # Start the additional thread
    # plot_thread.start()
    # uwb_slam.update_plot_obs()

    # ros_thread.join() # Wait for the thread to finish
    # plot_thread.join() # Wait for the thread to finish
    rclpy.spin(uwb_slam)
    pdb.set_trace()
    rclpy.shutdown()

if __name__ == '__main__':
    main()