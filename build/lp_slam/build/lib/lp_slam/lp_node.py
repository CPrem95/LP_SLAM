from lp_slam.src import ekf_funcs_lp as ekf
from matplotlib import pyplot as plt
import time
import numpy as np
import pickle
import math
import pdb
import math
import scipy.io

np.random.seed(10) # for reproducibility
plt.ion()

def main():
    plot = True
    print("Loading data...")
    # data = scipy.io.loadmat('/home/arms/paper3_ws/src/ekf_slam/lp_slam/lab.mat')
    # data_gt = scipy.io.loadmat('/home/arms/paper3_ws/src/ekf_slam/lp_slam/lab_gt.mat')

    data = scipy.io.loadmat('/home/arms/Documents/lab.mat')
    data_gt = scipy.io.loadmat('/home/arms/Documents/lab_gt.mat')
    # print(data.keys())

    fig = plt.figure()
    ax = fig.add_subplot(111) 
    ax.set_aspect('equal', adjustable='box')
    # plt.axis('equal')
    plt.grid(linestyle="--", color='black', alpha=0.3)
    ax.set_xlabel('x [mm]')
    ax.set_ylabel('y [mm]')
    ax.set_title('Odometry-based observations')
    plt.show()

    n_pt_landm = 50
    n_line_landm = 25

    winsize = 100
    cwinsize = 15
    stepsize = 3

    ini_landm_var = 1e6
    ini_landm_var = 1e4
    exp_pt_landm = n_pt_landm + 10
    exp_line_landm = n_line_landm + 10

    visLine_x = np.zeros([exp_line_landm, 2])
    visLine_y = np.zeros([exp_line_landm, 2])

    # obsLHS = np.zeros([6000, 5, 2])
    # obsRHS = np.zeros([6000, 5, 2])
    obsLHS = data['obsLHS']
    obsRHS = data['obsRHS']

    # odom = np.zeros([6000, 3])
    odom = data['odom']
    [s1, s2, s3] = obsLHS.shape

    obsLHS = np.concatenate((obsLHS, np.zeros([stepsize + 2*winsize, 5, 2])), axis=0)
    obsRHS = np.concatenate((obsRHS, np.zeros([stepsize + 2*winsize, 5, 2])), axis=0)

    odom = np.concatenate((odom, np.tile(odom[-1], (stepsize + 2*winsize, 1))), axis=0)

    new_obs = np.zeros([s1, 2*s2, s3])
    obs_count = np.zeros([1, exp_pt_landm + exp_line_landm])
    hist_i = np.zeros([1, exp_pt_landm + exp_line_landm])

    # projection matrix
    F = np.block([np.eye(3), np.zeros([3, 2*exp_pt_landm + 2*exp_line_landm])])
    ini_mu = np.zeros([3 + 2*exp_pt_landm + 2*exp_line_landm, 1])

    ini_cov_xx = np.zeros([3, 3])
    ini_cov_xp = np.zeros([3, 2*exp_pt_landm])
    ini_cov_px = np.zeros([2*exp_pt_landm, 3])
    ini_cov_pp = ini_landm_var * np.eye(2*exp_pt_landm)
    ini_cov_xl = np.zeros([3, 2*exp_line_landm])
    ini_cov_lx = np.zeros([2*exp_line_landm, 3])
    ini_cov_ml = np.zeros([2*exp_pt_landm, 2*exp_line_landm])
    ini_cov_lm = np.zeros([2*exp_line_landm, 2*exp_pt_landm])
    ini_cov_ll = np.eye(2*exp_line_landm)*ini_landm_var
    ini_cov = np.block([[ini_cov_xx, ini_cov_xp, ini_cov_xl],
                        [ini_cov_px, ini_cov_pp, ini_cov_ml],
                        [ini_cov_lx, ini_cov_lm, ini_cov_ll]])

    # (1,2) Regions
    nni_thresh_12 = 0.5
    N_inl_thresh_12 = 45
    L_len_thresh_12 = 600
    r_thresh_12 = 300
    th_thresh_12 = math.radians(10)
    thresh_12 = [nni_thresh_12, N_inl_thresh_12, L_len_thresh_12, r_thresh_12, th_thresh_12]

    # (1) Regions   
    nni_thresh_1 = 0.15
    N_inl_thresh_1 = 30
    L_len_thresh_1 = 350
    r_thresh_1 = 100
    th_thresh_1 = math.radians(10)
    thresh_1 = [nni_thresh_1, N_inl_thresh_1, L_len_thresh_1, r_thresh_1, th_thresh_1]
    N_LMs_1 = np.zeros([1,2])
    L_LMs_1 = np.zeros([10,3])
    P_LMs_1 = np.zeros([20,3])

    # (2) Regions
    nni_thresh_2 = 0.15
    N_inl_thresh_2 = 30
    L_len_thresh_2 = 350
    r_thresh_2 = r_thresh_1
    th_thresh_2 = th_thresh_1
    thresh_2 = [nni_thresh_2, N_inl_thresh_2, L_len_thresh_2, r_thresh_2, th_thresh_2]
    N_LMs_2 = np.zeros([1,2])
    L_LMs_2 = np.zeros([10,3])
    P_LMs_2 = np.zeros([20,3])

    # initial state
    sig = ini_cov
    mu = ini_mu

    # process noise
    R = np.array([[25, 0, 0],
                  [0, 25, 0],
                  [0, 0, 0.0001]])
    
    # measurement noise points
    Q_pts = np.array([[2500, 0],
                  [0, 0.25]])
    
    # measurement noise lines
    Q_lines = np.array([[2500, 0],
                  [0, 0.25]])
        
    N_pts = 0 # no observed points in the beginning
    N_line = 0 # no observed lines in the beginning

    I = []
    N = 0

    min_n_obs = 10
    min_pts = 9
    max_rad = 200

    path_samples = odom.shape[0]

    est_x = np.zeros([path_samples, 1])
    est_y = np.zeros([path_samples, 1])
    est_th = np.zeros([path_samples, 1])

    temp_mu1 = mu
    temp_mu2 = mu
    mu_bar = mu

    all_lhs_obs = [] # all left hand side observations
    all_rhs_obs = [] # all right hand side observations
    count_lhs_obs = np.zeros([1, path_samples], dtype=int) # count of left hand side observations
    count_rhs_obs = np.zeros([1, path_samples], dtype=int) # count of right hand side observations

    n_lhs = obsLHS.shape[1]
    n_rhs = obsRHS.shape[1]

    x_history = [0]
    y_history = [0]
    th_history = [0]

    odom1_history = []
    odom2_history = []

    obs_lin = np.zeros([33, 10, 8])
    obs_pts = np.zeros([33, 10, 3])

    count_i = 2

    stepend = path_samples - stepsize - winsize*2

    for i in range(path_samples):
        # print('i:', i)
        for j in range(n_lhs):
            if obsLHS[i, j, 0] != 0:
                tmp_lhs_obs = [odom[i, 0] + obsLHS[i, j, 0]*math.cos(obsLHS[i, j, 1] + odom[i, 2]), odom[i, 1] + obsLHS[i, j, 0]*math.sin(obsLHS[i, j, 1] + odom[i, 2])]
                all_lhs_obs.append(tmp_lhs_obs)
                count_lhs_obs[0, i] += 1
        
        for j in range(n_rhs):
            if obsRHS[i, j, 0] != 0:
                tmp_rhs_obs = [odom[i, 0] + obsRHS[i, j, 0]*math.cos(obsRHS[i, j, 1] + odom[i, 2]), odom[i, 1] + obsRHS[i, j, 0]*math.sin(obsRHS[i, j, 1] + odom[i, 2])]
                all_rhs_obs.append(tmp_rhs_obs)
                count_rhs_obs[0, i] += 1

    lhs_obs_xy = np.array(all_lhs_obs).reshape(-1, 2)
    rhs_obs_xy = np.array(all_rhs_obs).reshape(-1, 2)

    # Odometry plot
    odoms, = ax.plot([], [], linewidth = 2, c='k', marker='o', label='Odometry')
    fig.canvas.draw() 
    odom_dir, = ax.plot([], [], linewidth = 2, c='k')
    fig.canvas.draw()  
    # Raw observations plot
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
    plt_lines_lhs_R1, = ax.plot([], [], c='r', linewidth = 1, label='estimated lines R1')
    fig.canvas.draw()
    plt_lines_rhs_R1, = ax.plot([], [], c='r', linewidth = 1, label='estimated lines R1')
    fig.canvas.draw()
    plt_points_lhs_R1 = ax.scatter([], [], s = 30, edgecolors='r', facecolors= 'none', marker= 'o', label='estimated points R1')
    fig.canvas.draw()
    plt_points_rhs_R1 = ax.scatter([], [], s = 30, edgecolors='r', facecolors= 'none', marker= 'o', label='estimated points R1')
    fig.canvas.draw()
    # Globally filtered observations
    # plt_lines_all, = ax.plot([], [], c='purple', linewidth = 2, label='Final estimated lines')
    plt_lines_all = []
    for i in range(10):
        tmp_plt, = ax.plot([], [], c='purple', linewidth = 2, label='Final estimated lines')
        plt_lines_all.append(tmp_plt)
        fig.canvas.draw()
    plt_points_all = ax.scatter([], [], s = 30, edgecolors='k', facecolors= 'purple', marker= 'o', label='Final estimated points')
    fig.canvas.draw()
    
    # Legend 
    fig.legend(handles=[odoms, plt_obs_lhs_R12, plt_obs_lhs_R1, plt_lines_lhs_R12, plt_points_lhs_R12, plt_lines_lhs_R1, plt_points_lhs_R1], loc='upper right')
    fig.canvas.flush_events()
    # ax.relim()
    # ax.autoscale_view(True, True, True) 
    # xmin=X[:,0].min(); xmax=X[:,0].max()
    # ymin=X[:,1].min(); ymax=X[:,1].max()
    # ax.set_xlim(xmin-0.1*(xmax-xmin),xmax+0.1*(xmax-xmin))
    # ax.set_ylim(ymin-0.1*(ymax-ymin),ymax+0.1*(ymax-ymin))
    
    # ax.set_xlim(-4000, 10000)
    # ax.set_ylim(-6000, 10000)

    # time.sleep(5)
    winsize2 = int(1.5*winsize) # 1.5 times the window size for both regions

    for i in range(0, stepend, stepsize):
        print('i:', i)
        # if i > 111:
        #     print('i:', i)
        #     pdb.set_trace()

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

        rob_obs_pose = mu_bar[0:3]
        minNNI_12 = 50

        odom_R12 = odom[i:i+winsize2, :]
        odom_R1 = odom[i:i+winsize, :]

        vis_PL12 = True
        vis_PL1 = True
        vis_PL_all = True

        # LHS observations REGION 12
        vis = False
        d_thresh = 300
        N_LMs_lhs_12 = np.zeros([2], dtype=int)
        N_LMs_lhs_12, L_LMs_lhs_12 = ekf.lineExtract(obs_lhs_R12, odom_R12, rob_obs_pose, thresh_12, N_LMs_lhs_12, minNNI_12, vis) # Extract lines from the observations
        params = [15, 200] # [min_pts, max_rad]
        N_LMs_lhs_12, P_LMs_lhs_12, P_LMs_lhs_12_plt = ekf.pointExtract(obs_lhs_R12, L_LMs_lhs_12, N_LMs_lhs_12, d_thresh, params, odom_R12, rob_obs_pose, vis) # Extract points from the observations:: (points, L_LMs, N_LMs, d_thresh, params, odom_i, rob_obs_pose, fig):
        if vis_PL12:
            ekf.visPLs2(N_LMs_lhs_12, L_LMs_lhs_12, P_LMs_lhs_12_plt, i, plt_lines_lhs_R12, plt_points_lhs_R12, fig) # visualize on top of the odom based obs plot

        # RHS observations REGION 12
        vis = False
        d_thresh = 300
        N_LMs_rhs_12 = np.zeros([2], dtype=int)
        N_LMs_rhs_12, L_LMs_rhs_12 = ekf.lineExtract(obs_rhs_R12, odom_R12, rob_obs_pose, thresh_12, N_LMs_rhs_12, minNNI_12, vis) # Extract lines from the observations
        params = [15, 200] # [min_pts, max_rad]
        N_LMs_rhs_12, P_LMs_rhs_12, P_LMs_rhs_12_plt = ekf.pointExtract(obs_rhs_R12, L_LMs_rhs_12, N_LMs_rhs_12, d_thresh, params, odom_R12, rob_obs_pose, vis) # Extract points from the observations:: (points, L_LMs, N_LMs, d_thresh, params, odom_i, rob_obs_pose, fig):
        if vis_PL12:
            ekf.visPLs2(N_LMs_rhs_12, L_LMs_rhs_12, P_LMs_rhs_12_plt, i, plt_lines_rhs_R12, plt_points_rhs_R12, fig) # visualize on top of the odom based obs plot

        # LHS observations REGION 1
        vis = False
        d_thresh = 100
        N_LMs_lhs_1 = np.zeros([2], dtype=int)
        N_LMs_lhs_1, L_LMs_lhs_1 = ekf.lineExtract(obs_lhs_R1, odom_R1, rob_obs_pose, thresh_1, N_LMs_lhs_1, minNNI_12, vis) # Extract lines from the observations
        params = [10, 200] # [min_pts, max_rad]
        N_LMs_lhs_1, P_LMs_lhs_1, P_LMs_lhs_1_plt = ekf.pointExtract(obs_lhs_R1, L_LMs_lhs_1, N_LMs_lhs_1, d_thresh, params, odom_R1, rob_obs_pose, vis)
        if vis_PL1:
            ekf.visPLs2(N_LMs_lhs_1, L_LMs_lhs_1, P_LMs_lhs_1_plt, i, plt_lines_lhs_R1, plt_points_lhs_R1, fig)

        # RHS observations REGION 1
        vis = False
        d_thresh = 100
        N_LMs_rhs_1 = np.zeros([2], dtype=int)
        N_LMs_rhs_1, L_LMs_rhs_1 = ekf.lineExtract(obs_rhs_R1, odom_R1, rob_obs_pose, thresh_1, N_LMs_rhs_1, minNNI_12, vis) # Extract lines from the observations
        params = [10, 200] # [min_pts, max_rad]
        N_LMs_rhs_1, P_LMs_rhs_1, P_LMs_rhs_1_plt = ekf.pointExtract(obs_rhs_R1, L_LMs_rhs_1, N_LMs_rhs_1, d_thresh, params, odom_R1, rob_obs_pose, vis)
        if vis_PL1:
            ekf.visPLs2(N_LMs_rhs_1, L_LMs_rhs_1, P_LMs_rhs_1_plt, i, plt_lines_rhs_R1, plt_points_rhs_R1, fig)

        # Filter observations::
        obs_Lin, obs_Pt = ekf.createObs(N_LMs_lhs_1, L_LMs_lhs_1, P_LMs_lhs_1, N_LMs_lhs_12, L_LMs_lhs_12, 
                                        N_LMs_rhs_1, L_LMs_rhs_1, P_LMs_rhs_1, N_LMs_rhs_12, L_LMs_rhs_12, 
                                        mu, mu_bar, N_line, exp_pt_landm, obs_lhs_R12, obs_rhs_R12, visLine_x, visLine_y, fig, vis_PL_all, plt_lines_all, plt_points_all)

        if plot == True and 1:
            # Plot the observations
            plt_obs_lhs_R1.set_offsets(obs_lhs_R1)
            fig.canvas.draw() 
            plt_obs_rhs_R1.set_offsets(obs_rhs_R1)
            fig.canvas.draw()
            plt_obs_lhs_R12.set_offsets(obs_lhs_R12)
            fig.canvas.draw()
            plt_obs_rhs_R12.set_offsets(obs_rhs_R12)
            fig.canvas.draw()
            # Plot the odometry
            odoms.set_xdata(odom[i:i+stepsize, 0])
            odoms.set_ydata(odom[i:i+stepsize, 1])
            set_line(fig, odom_dir, odom1)
            # ax.relim()  # Recalculate the data limits
            # ax.autoscale()  # Adjust the axis limits
            
            # Adjust the axis limits
            set_limits(ax, np.concatenate((obs_lhs_R12, obs_rhs_R12), axis=0), odom[i:i+stepsize, :])
            fig.canvas.draw()
            fig.canvas.flush_events()

            # time.sleep(0.01)
            # for line in ax.lines:
            #     line.set_data([], [])
            for line in plt_lines_all:
                line.set_data([], [])

        # if i > 50:
        #     time.sleep(10000)
        #     break
    
    pdb.set_trace()
        

        
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

def set_line(fig, odom_dir, odom1):
    # Set the direction of the odometry
    odom_dir.set_xdata([odom1[0], odom1[0] + 200*math.cos(odom1[2])])
    odom_dir.set_ydata([odom1[1], odom1[1] + 200*math.sin(odom1[2])])
    fig.canvas.draw()
    fig.canvas.flush_events()









if __name__ == '__main__':
    main()
    
