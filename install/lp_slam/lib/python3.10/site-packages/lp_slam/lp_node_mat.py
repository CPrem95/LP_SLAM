# pylint: disable=C0103, C0116, W0611, C0302, C0301, C0303, C0114, C0321, R0914, R0912, R0915, W0105, W1515
## This program performs SLAM using the EKF algorithm
## This program works with a .mat file containing the observations and odometry data
## Use the save_data >> sv_data.py to save the data from ROS to a .mat file
# ## The observations are in the form of (range, bearing) for both left and right sides
## need 'obsLHS', 'obsRHS', 'odom' from the data
## May create a .mat file first using the

import time
import pickle
import math
import pdb
import argparse
import scipy.io
import rclpy
from lp_slam.src import ekf_funcs_lp as ekf
from rclpy.node import Node
# matplotlib.use('Agg')
from matplotlib import pyplot as plt
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.io import savemat
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import yaml

# Package share directory path
package_name = 'lp_slam'
package_share_directory = get_package_share_directory(package_name) # Now you have the package share directory path

# Dataset path
package_path = get_package_prefix(package_name)
len_pkg_path = len(package_path)
subpath_to_remove = 'install/' + package_name
len_subpath = len(subpath_to_remove)
# Remove the subpath
ws_path = package_path[:-len_subpath] 
ds_path = ws_path + 'dataset/'

np.random.seed(10) # for reproducibility
plt.ion()

class LP_SLAM_mat(Node):
    """Node for LP SLAM using the EKF algorithm"""
    def __init__(self):
        super().__init__('lp_slam_publisher')
        self.declare_parameters_from_file(package_share_directory + '/config/params.yaml')

        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('winsize', rclpy.Parameter.Type.INTEGER),
        #         ('stepsize', rclpy.Parameter.Type.INTEGER),
        #     ])

        self.ray_publisher = self.create_publisher(Float32MultiArray, '/my_scan', 10)
        self.mu_publisher = self.create_publisher(Float32MultiArray, '/mu', 10)
    
    def declare_parameters_from_file(self, params_file_path):
        with open(params_file_path, 'r') as file:
            params = yaml.safe_load(file)
            for key, value in params.get(self.get_name(), {}).get('ros__parameters', {}).items():
                self.declare_parameter(key, value)

def main(args=None):
    rclpy.init(args=args)
    slam_node = LP_SLAM_mat()
    # params = slam_node.get_parameters(['bool_value', 'int_number', 'float_number', 'str_text'])

    mu_msg = Float32MultiArray()
    ray_msg = Float32MultiArray()
    publish_data = False # publish for the occupancy grid

    parser = argparse.ArgumentParser(description='Passes the dataset name')
    # Adding arguments
    parser.add_argument('--ds_name', type=str, default="dataset", help='An optional argument to pass the dataset name')
    # Parsing arguments
    args = parser.parse_args()
    # Accessing arguments
    dataset_name = args.ds_name

    # Refer the Section: #### "Observation Extraction" #### and check its flags 
    # to turn on/off the visualization of the extracted lines and points

    print("Loading data...")
    # data_gt = scipy.io.loadmat('/home/arms/Documents/lab_gt.mat')
    try:
        data = scipy.io.loadmat(ds_path + str(dataset_name) + ".mat")
    except FileNotFoundError:
        print("File not found!")
        pause = input("Press any key to exit...")
        return
    
    # Figure for raw observations plot
    fig = plt.figure()
    ax = fig.add_subplot(111) 
    ax.set_aspect('equal', adjustable='box')
    # plt.axis('equal')
    plt.grid(linestyle="--", color='black', alpha=0.3)
    ax.set_xlabel('x [mm]')
    ax.set_ylabel('y [mm]')
    ax.set_title('Odometry-based observations')
    plt.show()

    # Figure for final estimated lines and points plot
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.set_aspect('equal', adjustable='box')
    plt.grid(linestyle="--", color='black', alpha=0.3)
    ax2.set_xlabel('x [mm]')
    ax2.set_ylabel('y [mm]')
    ax2.set_title('Final SLAM output')
    plt.show()

    ## PARAMETERS
    raw_plot = slam_node.get_parameter('raw_plot').get_parameter_value().bool_value # plot raw observations
    slam_plot = slam_node.get_parameter('slam_plot').get_parameter_value().bool_value # plot final SLAM output

    winsize = slam_node.get_parameter('winsize').get_parameter_value().integer_value # window size for a region
    stepsize = slam_node.get_parameter('stepsize').get_parameter_value().integer_value # step size for the SLAM update window
    r = slam_node.get_parameter('r').get_parameter_value().double_value # winsize ratio
    
    ini_pt_landm_var = slam_node.get_parameter('ini_pt_landm_var').get_parameter_value().integer_value # initial point landmark variance
    ini_ln_landm_var = slam_node.get_parameter('ini_ln_landm_var').get_parameter_value().integer_value # initial line landmark variance
    exp_pt_landm = slam_node.get_parameter('exp_pt_landm').get_parameter_value().integer_value # expected number of point landmarks
    exp_line_landm = slam_node.get_parameter('exp_line_landm').get_parameter_value().integer_value # expected number of line landmarks

    # (1,2) Regions
    N_inl_thresh_12 = slam_node.get_parameter('N_inl_thresh_12').get_parameter_value().integer_value # RANSAC inliers threshold
    L_len_thresh_12 = slam_node.get_parameter('L_len_thresh_12').get_parameter_value().integer_value # minimum line length
    r_thresh_12 = slam_node.get_parameter('r_thresh_12').get_parameter_value().integer_value # registers a line if it is not within 300 mm of the previous line
    th_thresh_12 = slam_node.get_parameter('th_thresh_12').get_parameter_value().double_value # registers a line if it is not within 1 degree of the previous line
    D_ransac_12 = slam_node.get_parameter('D_ransac_12').get_parameter_value().integer_value # RANSAC distance threshold
    thresh_12 = [N_inl_thresh_12, L_len_thresh_12, r_thresh_12, math.radians(th_thresh_12), D_ransac_12]

    # (1) Regions
    N_inl_thresh_1 = slam_node.get_parameter('N_inl_thresh_1').get_parameter_value().integer_value # RANSAC inliers threshold
    L_len_thresh_1 = slam_node.get_parameter('L_len_thresh_1').get_parameter_value().integer_value # minimum line length
    r_thresh_1 = slam_node.get_parameter('r_thresh_1').get_parameter_value().integer_value # registers a line if it is not within 300 mm of the previous line
    th_thresh_1 = slam_node.get_parameter('th_thresh_1').get_parameter_value().double_value # registers a line if it is not within 1 degree of the previous line
    D_ransac_1 = slam_node.get_parameter('D_ransac_1').get_parameter_value().integer_value # RANSAC distance threshold
    thresh_1 = [N_inl_thresh_1, L_len_thresh_1, r_thresh_1, math.radians(th_thresh_1), D_ransac_1]

    # M-distance thresholds
    alp_pt = slam_node.get_parameter('alp_pt').get_parameter_value().double_value # Mahalanobis distance threshold for point landmarks
    alp_ln = slam_node.get_parameter('alp_ln').get_parameter_value().double_value # Mahalanobis distance threshold for line landmarks
    alp_C = slam_node.get_parameter('alp_C').get_parameter_value().double_value # Mahalanobis distance threshold for the combined landmarks

    # process noise
    R = np.array([[slam_node.get_parameter('var_x').get_parameter_value().double_value, 0, 0],
                    [0, slam_node.get_parameter('var_y').get_parameter_value().double_value, 0],
                    [0, 0, slam_node.get_parameter('var_th').get_parameter_value().double_value]])
    
    # measurement noise points
    Q_pts = np.array([[slam_node.get_parameter('var_range').get_parameter_value().double_value, 0],
                    [0, slam_node.get_parameter('var_bearing').get_parameter_value().double_value]])
    
    # measurement noise lines
    Q_lines = np.array([[slam_node.get_parameter('var_ro').get_parameter_value().double_value, 0],
                    [0, slam_node.get_parameter('var_alpha').get_parameter_value().double_value]])
    
    ############################################################################################################
        
    visLine_x = np.zeros([exp_line_landm, 2])
    visLine_y = np.zeros([exp_line_landm, 2])

    # Load data
    obsLHS = data['obsLHS']
    obsRHS = data['obsRHS']
    odom = data['odom']

    [s1, s2, s3] = obsLHS.shape # shape of the observations
    # s1 : is the number of samples
    # s2 : is the number of observations in each sample
    # s3 : is the number of parameters for each observation e.g. xy
    
    odom = np.concatenate((np.zeros([1, 3]), odom), axis=0) # padding zeros to the beginning
    ''' for lab.mat
    obsLHS = np.concatenate((np.zeros([1, 5, 2]), obsLHS), axis=0) # padding zeros to the beginning
    obsRHS = np.concatenate((np.zeros([1, 5, 2]), obsRHS), axis=0) # padding zeros to the beginning
    '''
    # for lift.mat
    obsLHS = np.concatenate((np.zeros([1, s2, 2]), obsLHS), axis=0) # padding zeros to the beginning
    obsRHS = np.concatenate((np.zeros([1, s2, 2]), obsRHS), axis=0) # padding zeros to the beginning

    obsLHS = np.concatenate((obsLHS, np.zeros([stepsize + 2*winsize, s2, 2])), axis=0) # padding zeros to the end
    obsRHS = np.concatenate((obsRHS, np.zeros([stepsize + 2*winsize, s2, 2])), axis=0) # padding zeros to the end

    odom = np.concatenate((odom, np.tile(odom[-1], (stepsize + 2*winsize, 1))), axis=0) # padding zeros to the end

    obs_count = np.zeros([1, exp_pt_landm + exp_line_landm])
    hist_i = np.zeros(exp_pt_landm + exp_line_landm)

    # projection matrix
    F = np.block([np.eye(3), np.zeros([3, 2*exp_pt_landm + 2*exp_line_landm])])

    # initial state
    ini_mu = np.zeros([3 + 2*exp_pt_landm + 2*exp_line_landm, 1])

    # initial covariance matrix
    ini_cov_xx = np.zeros([3, 3])
    ini_cov_xp = np.zeros([3, 2*exp_pt_landm])
    ini_cov_px = np.zeros([2*exp_pt_landm, 3])
    ini_cov_pp = ini_pt_landm_var * np.eye(2*exp_pt_landm)
    ini_cov_xl = np.zeros([3, 2*exp_line_landm])
    ini_cov_lx = np.zeros([2*exp_line_landm, 3])
    ini_cov_ml = np.zeros([2*exp_pt_landm, 2*exp_line_landm])
    ini_cov_lm = np.zeros([2*exp_line_landm, 2*exp_pt_landm])
    ini_cov_ll = np.eye(2*exp_line_landm)*ini_ln_landm_var
    ini_cov = np.block([[ini_cov_xx, ini_cov_xp, ini_cov_xl],
                        [ini_cov_px, ini_cov_pp, ini_cov_ml],
                        [ini_cov_lx, ini_cov_lm, ini_cov_ll]])

    # initial state
    sig = ini_cov
    mu = ini_mu

    N_pts = 0 # no observed points in the beginning
    N_line = 0 # no observed lines in the beginning

    path_samples = odom.shape[0]

    mu_bar = mu

    all_lhs_obs = [] # all left hand side observations
    all_rhs_obs = [] # all right hand side observations
    all_lhs_obs_radial = [] # all left hand side observations:: range and bearing >>> needed for publishing the obs to the occ grid
    all_rhs_obs_radial = [] # all right hand side observations:: range and bearing >>> needed for publishing the obs to the occ grid
    count_lhs_obs = np.zeros([1, path_samples], dtype=int) # count of left hand side observations
    count_rhs_obs = np.zeros([1, path_samples], dtype=int) # count of right hand side observations

    n_lhs = obsLHS.shape[1]
    n_rhs = obsRHS.shape[1]

    x_history = [0]
    y_history = [0]
    th_history = [0]

    stepend = path_samples - stepsize - winsize*2

    for i in range(path_samples):
        # print('i:', i)
        for j in range(n_lhs):
            if obsLHS[i, j, 0] != 0:
                tmp_lhs_obs = [odom[i, 0] + obsLHS[i, j, 0]*math.cos(obsLHS[i, j, 1] + odom[i, 2]), odom[i, 1] + obsLHS[i, j, 0]*math.sin(obsLHS[i, j, 1] + odom[i, 2])]
                all_lhs_obs.append(tmp_lhs_obs)
                all_lhs_obs_radial.append([obsLHS[i, j, 0], obsLHS[i, j, 1]]) # >>> to be used for publishing the obs to the occ grid
                count_lhs_obs[0, i] += 1

        for j in range(n_rhs):
            if obsRHS[i, j, 0] != 0:
                tmp_rhs_obs = [odom[i, 0] + obsRHS[i, j, 0]*math.cos(obsRHS[i, j, 1] + odom[i, 2]), odom[i, 1] + obsRHS[i, j, 0]*math.sin(obsRHS[i, j, 1] + odom[i, 2])]
                all_rhs_obs.append(tmp_rhs_obs)
                all_rhs_obs_radial.append([obsRHS[i, j, 0], obsRHS[i, j, 1]]) # >>> to be used for publishing the obs to the occ grid
                count_rhs_obs[0, i] += 1

    all_lhs_obs_radial = np.array(all_lhs_obs_radial).reshape(-1, 2)
    all_rhs_obs_radial = np.array(all_rhs_obs_radial).reshape(-1, 2)

    lhs_obs_xy = np.array(all_lhs_obs).reshape(-1, 2)
    rhs_obs_xy = np.array(all_rhs_obs).reshape(-1, 2)

    ############################################################################################################
    ## Raw observations plot
    ############################################################################################################

    # Odometry plot
    odoms, = ax.plot([], [], linewidth = 2, c='k', marker='o', label='Odometry')
    fig.canvas.draw() 
    odom_dir, = ax.plot([], [], linewidth = 2, c='k')
    fig.canvas.draw()  
    # Raw observations plot
    plt_prev_obs_lhs_R1 = ax.scatter([], [], s=30, c='y', marker='*', label='prev_observations R1')
    fig.canvas.draw()
    plt_prev_obs_rhs_R1 = ax.scatter([], [], s=30, c='y', marker='*', label='prev_observations R1')
    fig.canvas.draw()
    plt_obs_lhs_R12 = ax.scatter([], [], s=30, c='g', marker='*', label='raw_observations R12')
    fig.canvas.draw()
    plt_obs_rhs_R12 = ax.scatter([], [], s=30, c='g', marker='*', label='raw_observations R12')
    fig.canvas.draw()
    plt_obs_lhs_R1 = ax.scatter([], [], s=30, c='b', marker='*', label='raw_observations R1')
    fig.canvas.draw()
    plt_obs_rhs_R1 = ax.scatter([], [], s=30, c='b', marker='*', label='raw_observations R1')
    fig.canvas.draw()
    # Estimated lines and points plot in REGIONS 1 2
    plt_lines_lhs_R12, = ax.plot([], [], c='k', linewidth = 1, label='estimated lines R12')
    fig.canvas.draw()
    plt_lines_rhs_R12, = ax.plot([], [], c='k', linewidth = 1, label='estimated lines R12')
    fig.canvas.draw()
    plt_points_lhs_R12 = ax.scatter([], [], s = 30, edgecolors='k', facecolors= 'none', marker= 'o', label='estimated points R12')
    fig.canvas.draw()
    plt_points_rhs_R12 = ax.scatter([], [], s = 30, edgecolors='k', facecolors= 'none', marker= 'o', label='estimated points R12')
    fig.canvas.draw()
    # Estimated lines and points plot in REGION 1
    plt_lines_lhs_R1, = ax.plot([], [], c='purple', linewidth = 1, label='estimated lines R1')
    fig.canvas.draw()
    plt_lines_rhs_R1, = ax.plot([], [], c='purple', linewidth = 1, label='estimated lines R1')
    fig.canvas.draw()
    plt_points_lhs_R1 = ax.scatter([], [], s = 30, edgecolors='purple', facecolors= 'none', marker= 'o', label='estimated points R1')
    fig.canvas.draw()
    plt_points_rhs_R1 = ax.scatter([], [], s = 30, edgecolors='purple', facecolors= 'none', marker= 'o', label='estimated points R1')
    fig.canvas.draw()
    # Globally filtered observations
    # plt_lines_all, = ax.plot([], [], c='purple', linewidth = 2, label='Final estimated lines')
    plt_lines_all = []
    for i in range(10):
        tmp_plt_ln_all, = ax.plot([], [], c='red', linewidth = 2, linestyle= '--', label='Final est lines')
        plt_lines_all.append(tmp_plt_ln_all)
        fig.canvas.draw()
    plt_points_all = ax.scatter([], [], s = 30, edgecolors='k', facecolors= 'red', marker= 'o', label='Final est points')
    fig.canvas.draw()
    
    # Legend 
    fig.legend(handles=[odoms, plt_obs_lhs_R12, plt_obs_lhs_R1, plt_lines_lhs_R12, plt_points_lhs_R12, plt_lines_lhs_R1, plt_points_lhs_R1, tmp_plt_ln_all, plt_points_all], loc='upper right')
    fig.canvas.flush_events()

    ############################################################################################################
    ## SLAM Plot
    ############################################################################################################
    
    # Estimated path plot
    raw_odom, = ax2.plot([], [], linewidth = 2, c='k', linestyle= '--', label='Odometry')
    fig2.canvas.draw()
    est_path, = ax2.plot([], [], linewidth = 2, c='b', label='Estimated path')
    fig2.canvas.draw()
    # Estimated pose
    est_pose = ax2.scatter([], [], s=30, c='navy', marker='o', label='Current pose')
    fig2.canvas.draw()
    # Estimated point landmarks plot
    est_pt_lms = ax2.scatter([], [], s=30, c='red', marker='o', label='Estimated point landmarks')
    fig2.canvas.draw()
    # Estimated line landmarks plot
    est_ln_lms = []
    for i in range(exp_line_landm):
        tmp_plt_ln_all, = ax2.plot([], [], c='red', linewidth = 3, linestyle= '-', label='Estimated line landmarks')
        est_ln_lms.append(tmp_plt_ln_all)
        fig.canvas.draw()

    # Legend
    fig2.legend(handles=[raw_odom, est_path, est_pt_lms, tmp_plt_ln_all], loc='upper right')
    fig2.canvas.flush_events()
    
    ############################################################################################################
    ## SLAM
    ############################################################################################################

    ## Define the window size for BOTH REGIONS R12
    winsize2 = int(r*winsize) # 1.5 times the window size for both regions

    ## main loop
    pdb.set_trace()
    empty_array = np.array([[0,0]]) # Used in the visualization points in regions 1 and 2 >>> has to be replaced by P_LMs_*hs_12_plt if required

    vis = False # visualize regressed lines
    # REGION 12 line and point visualization
    # REGION 1 line and point visualization
    # FINAL All line and point visualization
    vis_PL12, vis_PL1, vis_PL_all = True, True, True
    if not raw_plot: 
        vis_PL12, vis_PL1, vis_PL_all = False, False, False

    for i in range(0, stepend, stepsize):
        print('\n###########################################################################')
        print('i:', i)

        '''
        if i == 9001: # 8100 Load the saved data
            save = False
            load = True
            if save:
                with open("mu.pickle", "wb") as f:
                    pickle.dump(mu, f)
                with open("sig.pickle", "wb") as f:
                    pickle.dump(sig, f)
                with open("N_pts.pickle", "wb") as f:
                    pickle.dump(N_pts, f)
                with open("N_line.pickle", "wb") as f:
                    pickle.dump(N_line, f)
                with open("obs_count.pickle", "wb") as f:
                    pickle.dump(obs_count, f)
                with open("hist_i.pickle", "wb") as f:
                    pickle.dump(hist_i, f)
                with open("visLine_x.pickle", "wb") as f:
                    pickle.dump(visLine_x, f)
                with open("visLine_y.pickle", "wb") as f:
                    pickle.dump(visLine_y, f)
                with open("x_history.pickle", "wb") as f:
                    pickle.dump(x_history, f)
                with open("y_history.pickle", "wb") as f:
                    pickle.dump(y_history, f)
                with open("th_history.pickle", "wb") as f:
                    pickle.dump(th_history, f)
            if load:
                with open("mu.pickle", "rb") as f:
                    mu = pickle.load(f)
                with open("sig.pickle", "rb") as f:
                    sig = pickle.load(f)
                with open("N_pts.pickle", "rb") as f:
                    N_pts = pickle.load(f)
                with open("N_line.pickle", "rb") as f:
                    N_line = pickle.load(f)
                with open("obs_count.pickle", "rb") as f:
                    obs_count = pickle.load(f)
                with open("hist_i.pickle", "rb") as f:
                    hist_i = pickle.load(f)
                with open("visLine_x.pickle", "rb") as f:
                    visLine_x = pickle.load(f)
                with open("visLine_y.pickle", "rb") as f:
                    visLine_y = pickle.load(f)
                with open("x_history.pickle", "rb") as f:
                    x_history = pickle.load(f)
                with open("y_history.pickle", "rb") as f:
                    y_history = pickle.load(f)
                with open("th_history.pickle", "rb") as f:
                    th_history = pickle.load(f)
            pdb.set_trace()
        if i == 9001: # Save the data
            save = True
            load = False
            if save:
                with open("mu.pickle", "wb") as f:
                    pickle.dump(mu, f)
                with open("sig.pickle", "wb") as f:
                    pickle.dump(sig, f)
                with open("N_pts.pickle", "wb") as f:
                    pickle.dump(N_pts, f)
                with open("N_line.pickle", "wb") as f:
                    pickle.dump(N_line, f)
                with open("obs_count.pickle", "wb") as f:
                    pickle.dump(obs_count, f)
                with open("hist_i.pickle", "wb") as f:
                    pickle.dump(hist_i, f)
                with open("visLine_x.pickle", "wb") as f:
                    pickle.dump(visLine_x, f)
                with open("visLine_y.pickle", "wb") as f:
                    pickle.dump(visLine_y, f)
                with open("x_history.pickle", "wb") as f:
                    pickle.dump(x_history, f)
                with open("y_history.pickle", "wb") as f:
                    pickle.dump(y_history, f)
                with open("th_history.pickle", "wb") as f:
                    pickle.dump(th_history, f)
            if load:
                with open("mu.pickle", "rb") as f:
                    mu = pickle.load(f)
                with open("sig.pickle", "rb") as f:
                    sig = pickle.load(f)
                with open("N_pts.pickle", "rb") as f:
                    N_pts = pickle.load(f)
                with open("N_line.pickle", "rb") as f:
                    N_line = pickle.load(f)
                with open("obs_count.pickle", "rb") as f:
                    obs_count = pickle.load(f)
                with open("hist_i.pickle", "rb") as f:
                    hist_i = pickle.load(f)
                with open("visLine_x.pickle", "rb") as f:
                    visLine_x = pickle.load(f)
                with open("visLine_y.pickle", "rb") as f:
                    visLine_y = pickle.load(f)
                with open("x_history.pickle", "rb") as f:
                    x_history = pickle.load(f)
                with open("y_history.pickle", "rb") as f:
                    y_history = pickle.load(f)
                with open("th_history.pickle", "rb") as f:
                    th_history = pickle.load(f)
            pdb.set_trace()
        '''

        # if i == 18000:
        #     pdb.set_trace()
        #     raw_plot = True
        #     slam_plot = True
        #     vis_PL12, vis_PL1, vis_PL_all = True, True, True

        # Find indices for the observations in the current window || REGION 1
        start_ind_lhs_R1 = sum(count_lhs_obs[0, 0:i])
        end_ind_lhs_R1 = sum(count_lhs_obs[0, 0:i+winsize])
        start_ind_rhs_R1 = sum(count_rhs_obs[0, 0:i])
        end_ind_rhs_R1 = sum(count_rhs_obs[0, 0:i+winsize])

        # Slice the observations in the current window using the indices above || REGION 1
        obs_lhs_R1 = lhs_obs_xy[start_ind_lhs_R1:end_ind_lhs_R1][:]
        obs_rhs_R1 = rhs_obs_xy[start_ind_rhs_R1:end_ind_rhs_R1][:]

        # Find indices for the observations in the current window || REGION 12
        start_ind_lhs_R12 = sum(count_lhs_obs[0, 0:i])
        end_ind_lhs_R12 = sum(count_lhs_obs[0, 0:i+winsize2])
        start_ind_rhs_R12 = sum(count_rhs_obs[0, 0:i])
        end_ind_rhs_R12 = sum(count_rhs_obs[0, 0:i+winsize2])

        # Slice the observations in the current window using the indices above || REGION 12
        obs_lhs_R12 = lhs_obs_xy[start_ind_lhs_R12:end_ind_lhs_R12][:]
        obs_rhs_R12 = rhs_obs_xy[start_ind_rhs_R12:end_ind_rhs_R12][:]

        # To find isolated points (check func: ekf_unknown_correction_LP)
        if i > winsize2: 
            all_start_ind_lhs_R1 = sum(count_lhs_obs[0, 0:(i - winsize)])
            all_start_ind_rhs_R1 = sum(count_rhs_obs[0, 0:(i - winsize)])

            all_obs_lhs = lhs_obs_xy[all_start_ind_lhs_R1:end_ind_lhs_R12][:]
            all_obs_rhs = rhs_obs_xy[all_start_ind_rhs_R1:end_ind_rhs_R12][:]
        else:
            all_obs_lhs = lhs_obs_xy[0:end_ind_lhs_R12][:]
            all_obs_rhs = rhs_obs_xy[0:end_ind_rhs_R12][:]                             

        ############################################################################################################
        ## EKF Prediction 
        ############################################################################################################
        # start and end odometry
        odom0 = odom[i, :]
        odom1 = odom[i+stepsize, :]
        # control input
        u = ekf.odom2u(odom0, odom1)
        # prediction
        mu_bar, sig_bar = ekf.ekf_unkown_predict(mu, sig, u, R, F)

        # Update the observations based on the predicted odometry
        del_odom = odom0 - mu[0:3].reshape(1, -1)[0] # difference between the predicted and the actual odometry
        # print('del_odom:', del_odom)
        # obs_lhs_R1 = obs_lhs_R1 - del_odom[:2]
        # obs_rhs_R1 = obs_rhs_R1 - del_odom[:2]
        # obs_lhs_R12 = obs_lhs_R12 - del_odom[:2]
        # obs_rhs_R12 = obs_rhs_R12 - del_odom[:2]
        
        obs_lhs_R1 = obs_lhs_R1 - odom1[:2]
        obs_rhs_R1 = obs_rhs_R1 - odom1[:2]
        obs_lhs_R12 = obs_lhs_R12 - odom1[:2]
        obs_rhs_R12 = obs_rhs_R12 - odom1[:2]
        all_obs_lhs = all_obs_lhs - odom1[:2]
        all_obs_rhs = all_obs_rhs - odom1[:2]

        # Rotate the observations to the robot frame
        res_mu_bar = mu_bar[0:2].reshape(1, -1)[0]
        # obs_lhs_R1 = ekf.rotate_points(obs_lhs_R1, -del_odom[2]) + res_mu_bar
        # obs_rhs_R1 = ekf.rotate_points(obs_rhs_R1, -del_odom[2]) + res_mu_bar
        # obs_lhs_R12 = ekf.rotate_points(obs_lhs_R12, -del_odom[2]) + res_mu_bar
        # obs_rhs_R12 = ekf.rotate_points(obs_rhs_R12, -del_odom[2]) + res_mu_bar

        obs_lhs_R1 = ekf.rotate_points(obs_lhs_R1, del_odom[2]) + res_mu_bar
        obs_rhs_R1 = ekf.rotate_points(obs_rhs_R1, del_odom[2]) + res_mu_bar
        obs_lhs_R12 = ekf.rotate_points(obs_lhs_R12, del_odom[2]) + res_mu_bar
        obs_rhs_R12 = ekf.rotate_points(obs_rhs_R12, del_odom[2]) + res_mu_bar
        all_obs_lhs = ekf.rotate_points(all_obs_lhs, del_odom[2]) + res_mu_bar
        all_obs_rhs = ekf.rotate_points(all_obs_rhs, del_odom[2]) + res_mu_bar

        if publish_data: 

            # obs_lhs_R1_radial = all_lhs_obs_radial[start_ind_lhs_R1:end_ind_lhs_R1][:]
            # obs_rhs_R1_radial = all_rhs_obs_radial[start_ind_rhs_R1:end_ind_rhs_R1][:]
            # ray_msg_data = np.concatenate((obs_lhs_R1_radial, obs_rhs_R1_radial), axis = 0).reshape(1, -1)

            ray_msg_data = np.concatenate((obs_lhs_R1/1000, obs_rhs_R1/1000), axis = 0).reshape(1, -1)
            # print(ray_msg_data)
            ray_msg.data = ray_msg_data[0]
            slam_node.ray_publisher.publish(ray_msg)

            mu_msg.data = [mu[0][0]/1000, mu[1][0]/1000, mu[2][0]]
            slam_node.mu_publisher.publish(mu_msg)
        
        ############################################################################################################
        ## Observation Extraction
        ############################################################################################################
        # rob_obs_pose = mu_bar[0:3]
        rob_obs_pose = mu_bar[0:3].reshape(-1, 1)

        odom_R12 = odom[i:i+winsize2, :]
        odom_R1 = odom[i:i+winsize, :]

        # LHS observations REGION 12
        # vis = False # visualize regressed lines
        # print('Region 12 LHS observations:')
        # d_thresh = 300
        N_LMs_lhs_12 = np.zeros([2], dtype=int)
        N_LMs_lhs_12, L_LMs_lhs_12, _ = ekf.lineExtract(obs_lhs_R12, odom_R12, rob_obs_pose, thresh_12, N_LMs_lhs_12, vis) # Extract lines from the observations
        if vis_PL12:
            ekf.visPLs2(N_LMs_lhs_12, L_LMs_lhs_12, empty_array, plt_lines_lhs_R12, plt_points_lhs_R12, fig) # visualize on top of the odom based obs plot

        # RHS observations REGION 12
        # vis = False # visualize regressed lines
        # print('Region 12 RHS observations:')
        # d_thresh = 300
        N_LMs_rhs_12 = np.zeros([2], dtype=int)
        N_LMs_rhs_12, L_LMs_rhs_12, _ = ekf.lineExtract(obs_rhs_R12, odom_R12, rob_obs_pose, thresh_12, N_LMs_rhs_12, vis) # Extract lines from the observations
        if vis_PL12:
            ekf.visPLs2(N_LMs_rhs_12, L_LMs_rhs_12, empty_array, plt_lines_rhs_R12, plt_points_rhs_R12, fig) # visualize on top of the odom based obs plot

        # LHS observations REGION 1
        # vis = False
        # print('Region 1 LHS observations:')
        N_LMs_lhs_1 = np.zeros([2], dtype=int)
        N_LMs_lhs_1, L_LMs_lhs_1, out_Pts_lhs_R1 = ekf.lineExtract(obs_lhs_R1, odom_R1, rob_obs_pose, thresh_1, N_LMs_lhs_1, vis) # Extract lines from the observations
        params = [10, 200]; d_thresh = 100 # [min_pts, max_rad], threshold distance between line and a point
        N_LMs_lhs_1, P_LMs_lhs_1, P_LMs_lhs_1_plt = ekf.pointExtract(out_Pts_lhs_R1, L_LMs_lhs_1, N_LMs_lhs_1, d_thresh, params, odom_R1, rob_obs_pose, vis)
        if vis_PL1:
            ekf.visPLs2(N_LMs_lhs_1, L_LMs_lhs_1, P_LMs_lhs_1_plt, plt_lines_lhs_R1, plt_points_lhs_R1, fig)

        # RHS observations REGION 1
        # vis = False
        # print('Region 1 RHS observations:')
        N_LMs_rhs_1 = np.zeros([2], dtype=int)
        N_LMs_rhs_1, L_LMs_rhs_1, out_Pts_rhs_R1 = ekf.lineExtract(obs_rhs_R1, odom_R1, rob_obs_pose, thresh_1, N_LMs_rhs_1, vis) # Extract lines from the observations
        params = [10, 200]; d_thresh = 100 # [min_pts, max_rad], threshold distance between line and a point
        N_LMs_rhs_1, P_LMs_rhs_1, P_LMs_rhs_1_plt = ekf.pointExtract(out_Pts_rhs_R1, L_LMs_rhs_1, N_LMs_rhs_1, d_thresh, params, odom_R1, rob_obs_pose, vis)
        if vis_PL1:
            ekf.visPLs2(N_LMs_rhs_1, L_LMs_rhs_1, P_LMs_rhs_1_plt, plt_lines_rhs_R1, plt_points_rhs_R1, fig)

        # Filter observations::
        obs_Lin, obs_Pt = ekf.createObs(N_LMs_lhs_1, L_LMs_lhs_1, P_LMs_lhs_1, N_LMs_lhs_12, L_LMs_lhs_12, 
                                        N_LMs_rhs_1, L_LMs_rhs_1, P_LMs_rhs_1, N_LMs_rhs_12, L_LMs_rhs_12, 
                                        rob_obs_pose, mu, mu_bar, N_line, exp_pt_landm, visLine_x, visLine_y, vis_PL_all, plt_lines_all, plt_points_all) 

        # Plot the observations
        if raw_plot or i%450 == 0:
            plt_prev_obs_lhs_R1.set_offsets(all_obs_lhs)
            fig.canvas.draw()
            plt_prev_obs_rhs_R1.set_offsets(all_obs_rhs)
            fig.canvas.draw()
            plt_obs_lhs_R1.set_offsets(obs_lhs_R1)
            fig.canvas.draw() 
            plt_obs_rhs_R1.set_offsets(obs_rhs_R1)
            fig.canvas.draw()
            plt_obs_lhs_R12.set_offsets(obs_lhs_R12)
            fig.canvas.draw()
            plt_obs_rhs_R12.set_offsets(obs_rhs_R12)
            fig.canvas.draw()
            # Plot the odometry
            odoms.set_xdata(odom[i:i+stepsize, 0] - del_odom[0])
            odoms.set_ydata(odom[i:i+stepsize, 1] - del_odom[1])
            set_odom_dir(fig, odom_dir, odom1 - del_odom)

            # ax.relim()  # Recalculate the data limits
            # ax.autoscale()  # Adjust the axis limits
            
            # Adjust the axis limits

            set_limits(ax, np.concatenate((obs_lhs_R12, obs_rhs_R12, [[mu[0][0], mu[1][0]]]), axis=0), mu[0:2])
            fig.canvas.draw()
            fig.canvas.flush_events()

            # time.sleep(0.01)
            for line in plt_lines_all:
                line.set_data([], [])

        ############################################################################################################
        ## EKF Update
        ############################################################################################################
        mu, sig, N_pts, N_line, hist_i, obs_count, visLine_x, visLine_y = ekf.EKF_unknown_correction_LP(mu_bar, sig_bar, obs_Pt, obs_Lin, all_obs_lhs, all_obs_rhs, Q_pts, Q_lines, i, hist_i, obs_count, exp_pt_landm, exp_line_landm, N_pts, N_line, visLine_x, visLine_y, alp_pt, alp_ln, alp_C) # alp_pt, alp_line
        # print("hist_i_pt: ", hist_i[0:10])
        # print("hist_i_ln: ", hist_i[50:60])
        print("\nObserved LMs: ", N_pts, N_line)

        x_history.append(mu[0][0])
        y_history.append(mu[1][0])
        th_history.append(mu[2][0])

        ############################################################################################################
        ## SLAM Animation
        ############################################################################################################
        if slam_plot or i%150 == 0:
        # if slam_plot:
            # Plot the raw odometry
            raw_odom.set_xdata(odom[0:i+stepsize, 0])
            raw_odom.set_ydata(odom[0:i+stepsize, 1])
            # Plot the estimated path
            est_pose.set_offsets([mu[0][0], mu[1][0]]) # estimated pose
            est_path.set_xdata(x_history)
            est_path.set_ydata(y_history)
            # Plot the estimated landmarks
            est_pt_lms.set_offsets(mu[3:3+2*N_pts].reshape(-1, 2))
            
            # for i in range(N_line):
            #     est_ln_lms[i].set_xdata([mu[3+2*exp_pt_landm + 2*i][0], mu[3+2*exp_pt_landm + 2*i+1][0]])
            #     est_ln_lms[i].set_ydata([mu[3+2*exp_pt_landm + 2*i][1], mu[3+2*exp_pt_landm + 2*i+1][1]])

            # Adjust the axis limits
            set_limits2(ax2, [x_history, y_history], odom[0:i+stepsize, :])
            fig2.canvas.draw()

            # Plot the estimated landmarks
            # Lines
            lnstart = exp_pt_landm*2 + 3
            pi = math.pi
            for lin_i in range (N_line):
                
                line_r = mu[2*lin_i + lnstart]
                line_th = mu[2*lin_i + lnstart + 1]

                line_m = math.tan(pi/2 + line_th)
                line_c = line_r/math.sin(line_th)

                if abs(abs(line_th) - pi/2) < pi/4:
                    x = visLine_x[lin_i, :]
                    y = line_m*x + line_c
                else:
                    y = visLine_y[lin_i, :]
                    x = (y - line_c)/line_m

                est_ln_lms[lin_i].set_xdata(x)
                est_ln_lms[lin_i].set_ydata(y)
                fig.canvas.draw()
            
            # Points
            est_pt_lms.set_offsets(mu[3:3+2*N_pts].reshape(-1, 2))
            fig.canvas.draw()

            fig2.canvas.flush_events()
    pdb.set_trace()
    
    saveFinal = True
    if saveFinal: # Save the data
        savemat('final_results.mat', {'mu': mu, 'sig': sig, 'N_pts': N_pts, 'N_line': N_line, 'obs_count': obs_count, 'hist_i': hist_i, 'visLine_x': visLine_x, 'visLine_y': visLine_y, 'x_history': x_history, 'y_history': y_history, 'th_history': th_history, 'odom': odom})
        print("Saved the final data..!!")

## User-defined FUNCTIONS 
def set_limits(ax, X, mu):
    xmin=min(min(X[:, 0]), mu[0]); xmax=max(max(X[:, 0]), mu[0])
    ymin=min(min(X[:, 1]), mu[1]); ymax=max(max(X[:, 1]), mu[1])
    ax.set_xlim(xmin - 1000, xmax + 1000)
    ax.set_ylim(ymin - 1000, ymax + 1000)

def set_limits2(ax, data, odom):
    tmp_xmin=min(data[0]); tmp_xmax=max(data[0])
    tmp_ymin=min(data[1]); tmp_ymax=max(data[1])
    tmp_xmin2 = min(odom[:, 0]); tmp_xmax2 = max(odom[:, 0])
    tmp_ymin2 = min(odom[:, 1]); tmp_ymax2 = max(odom[:, 1])

    xmin = min(tmp_xmin, tmp_xmin2); xmax = max(tmp_xmax, tmp_xmax2)
    ymin = min(tmp_ymin, tmp_ymin2); ymax = max(tmp_ymax, tmp_ymax2)

    ax.set_xlim(xmin - 3000, xmax + 3000)
    ax.set_ylim(ymin - 3000, ymax + 3000)

def set_odom_dir(fig, odom_dir, odom1):
    # Set the direction of the odometry
    odom_dir.set_xdata([odom1[0], odom1[0] + 200*math.cos(odom1[2])])
    odom_dir.set_ydata([odom1[1], odom1[1] + 200*math.sin(odom1[2])])
    fig.canvas.draw()
    fig.canvas.flush_events()

def remove_subpath(full_path, subpath_to_remove):
    # Remove the subpath from the full path
    # This approach assumes that the subpath is at the end of the full path
    if full_path.endswith(subpath_to_remove):
        # Calculate the length of the subpath plus the leading '/'
        length_to_remove = len(subpath_to_remove) + 1  # +1 for the leading '/'
        return full_path[:-length_to_remove]
    return full_path

if __name__ == '__main__':
    main()
