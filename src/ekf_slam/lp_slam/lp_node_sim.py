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
    raw_plot = True #  Plot raw observations
    slam_plot = True # Plot final SLAM output
    
    # Figure for final estimated lines and points plot
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.set_aspect('equal', adjustable='box')
    plt.grid(linestyle="--", color='black', alpha=0.3)
    ax2.set_xlabel('x [mm]')
    ax2.set_ylabel('y [mm]')
    ax2.set_title('Final SLAM output')
    plt.show()

    # Refer the Section: #### "Observation Extraction" #### and check its flags 
    # to turn on/off the visualization of the extracted lines and points

    print("Loading data...")
    # data = scipy.io.loadmat('/home/arms/paper3_ws/src/ekf_slam/lp_slam/lab.mat')
    # data_gt = scipy.io.loadmat('/home/arms/paper3_ws/src/ekf_slam/lp_slam/lab_gt.mat')

    simdata = scipy.io.loadmat('/home/arms/Documents/robot.mat')

    # Load sim data
    sim_lin = simdata['obs_lin']
    sim_pts = simdata['obs_pts']
    sim_odom = simdata['odom']

    # Parameters

    ini_pt_landm_var = 1e6 # initial point landmark variance
    ini_ln_landm_var = 1e4 # initial line landmark variance
    exp_pt_landm = 50 # expected number of point landmarks
    exp_line_landm = 25 # expected number of line landmarks

    visLine_x = np.zeros([exp_line_landm, 2])
    visLine_y = np.zeros([exp_line_landm, 2])

    [s1, s2, s3] = sim_lin.shape # shape of the observations
    # s1 : is the number of samples
    # s2 : is the number of observations in each sample
    # s3 : is the number of parameters for each observation e.g. xy

    obs_count = np.zeros([1, exp_pt_landm + exp_line_landm])
    hist_i = np.zeros([1, exp_pt_landm + exp_line_landm])

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

    # process noise
    R = np.array([[0.25, 0, 0],
                  [0, 0.25, 0],
                  [0, 0, 0.004]])
    
    # measurement noise points
    Q_pts = np.array([[0.1, 0],
                  [0, 0.25]])
    
    # measurement noise lines
    Q_lines = np.array([[0.1, 0],
                  [0, 0.25]])
        
    N_pts = 0 # no observed points in the beginning
    N_line = 0 # no observed lines in the beginning

    # min_n_obs = 10 # used in DBSCAN-based filtering, not used
    # min_pts = 9
    # max_rad = 200

    path_samples = sim_odom.shape[0]

    est_x = np.zeros([path_samples, 1])
    est_y = np.zeros([path_samples, 1])
    est_th = np.zeros([path_samples, 1])

    mu_bar = mu

    all_lhs_obs = [] # all left hand side observations
    all_rhs_obs = [] # all right hand side observations
    count_lhs_obs = np.zeros([1, path_samples], dtype=int) # count of left hand side observations
    count_rhs_obs = np.zeros([1, path_samples], dtype=int) # count of right hand side observations

    x_history = [0]
    y_history = [0]
    th_history = [0]

    odom1_history = []
    odom2_history = []

    obs_lin = np.zeros([33, 10, 8])
    obs_pts = np.zeros([33, 10, 3])

    ############################################################################################################
    ## SLAM Plot
    ############################################################################################################
    
    # Estimated path plot
    raw_odom, = ax2.plot([], [], linewidth = 2, c='k', label='Raw path')
    fig2.canvas.draw()
    est_path, = ax2.plot([], [], linewidth = 2, c='b', label='Est path')
    fig2.canvas.draw()
    # Estimated point landmarks plot
    est_pt_lms = ax2.scatter([], [], s=30, c='red', marker='o', label='Est Pt LMs')
    fig2.canvas.draw()
    # Estimated line landmarks plot
    est_ln_lms = []
    for i in range(20):
        tmp_plt_ln_all, = ax2.plot([], [], c='red', linewidth = 2, linestyle= '-', label='Est Ln LMs')
        est_ln_lms.append(tmp_plt_ln_all)
        fig2.canvas.draw()

    # Legend
    fig2.legend(handles=[raw_odom, est_path, est_pt_lms, tmp_plt_ln_all], loc='upper right')
    fig2.canvas.flush_events()
    
    ############################################################################################################
    ## SLAM
    ############################################################################################################

    for i in range(1, path_samples):
        print('\n----------------------------------------------------------------------------')
        print('i:', i)
        # if i > 111:
        #     print('i:', i)
        #     pdb.set_trace()

        ############################################################################################################
        ## EKF Prediction 
        ############################################################################################################
        # start and end odometry
        odom0 = sim_odom[i-1, :]
        odom1 = sim_odom[i, :]
        # control input
        u = ekf.odom2u(odom0, odom1)
        # prediction
        mu_bar, sig_bar = ekf.ekf_unkown_predict(mu, sig, u, R, F)

        ############################################################################################################
        ## EKF Update
        ############################################################################################################
        mu, sig, N_pts, N_line, hist_i, obs_count, visLine_x, visLine_y = ekf.EKF_unknown_correction_LP(mu_bar, sig_bar, sim_pts[i], sim_lin[i], Q_pts, Q_lines, i, hist_i, obs_count, exp_pt_landm, exp_line_landm, N_pts, N_line, visLine_x, visLine_y, 1, 5)
        print("Observed LMs: ", N_pts, N_line)
        x_history.append(mu[0][0])
        y_history.append(mu[1][0])
        th_history.append(mu[2][0])

        ############################################################################################################
        ## SLAM Animation
        ############################################################################################################
        if slam_plot:
            # Plot the raw odometry
            raw_odom.set_xdata(sim_odom[0:i, 0])
            raw_odom.set_ydata(sim_odom[0:i, 1])
            # Plot the estimated path
            est_path.set_xdata(x_history)
            est_path.set_ydata(y_history)
            # Plot the estimated landmarks
            # print("mu:", mu[3:3+2*N_pts].reshape(-1, 2))
            # print("mu:", mu)
            est_pt_lms.set_offsets(mu[3:3+2*N_pts].reshape(-1, 2))
            # for i in range(N_line):
            #     est_ln_lms[i].set_xdata([mu[3+2*exp_pt_landm + 2*i][0], mu[3+2*exp_pt_landm + 2*i+1][0]])
            #     est_ln_lms[i].set_ydata([mu[3+2*exp_pt_landm + 2*i][1], mu[3+2*exp_pt_landm + 2*i+1][1]])

            # Adjust the axis limits
            set_limits(ax2, np.array([[-10,-10], [10,10]]), sim_odom[i:i+100, :])
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
                fig2.canvas.draw()
            
            # Points
            est_pt_lms.set_offsets(mu[3:3+2*N_pts].reshape(-1, 2))
            fig2.canvas.draw()

            fig2.canvas.flush_events()


    pdb.set_trace()
        

        
def set_limits(ax, X, odom = []):
    # odom = np.array(odom).reshape(-1, 2)
    # print("odom:", odom)
    # print("X:", X)
    # Sets the axis limits to include all the data points in X
    data = np.concatenate((X, odom[:, 0:2]), axis=0)
    xmin=min(data[:, 0]); xmax=max(data[:, 0])
    ymin=min(data[:, 1]); ymax=max(data[:, 1])
    ax.set_xlim(xmin-0.1*(xmax-xmin) - 1, xmax+0.1*(xmax-xmin) + 1)
    ax.set_ylim(ymin-0.1*(ymax-ymin) - 1, ymax+0.1*(ymax-ymin) + 1)

def set_odom_dir(fig, odom_dir, odom1):
    # Set the direction of the odometry
    odom_dir.set_xdata([odom1[0], odom1[0] + 200*math.cos(odom1[2])])
    odom_dir.set_ydata([odom1[1], odom1[1] + 200*math.sin(odom1[2])])
    fig.canvas.draw()
    fig.canvas.flush_events()









if __name__ == '__main__':
    main()
    
