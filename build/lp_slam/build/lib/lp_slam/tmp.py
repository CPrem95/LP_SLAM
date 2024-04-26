from lp_slam.src import env_funcs_anim as efa
from lp_slam.src import ekf_funcs_anchors as eka
from matplotlib import pyplot as plt
import time
import numpy as np
import pickle
import math
import pdb
from scipy import io
from lp_slam.src import gslam2d as g2o

plt.ion()

def main():
    fig = plt.figure()
    ax = fig.add_subplot(111) 

    save = 1
    path_samples = 1000
    n_landm = 20
    print('Hi from ekf_slam animation.')
    # create the environment
     
    if save:
        env, anchors, ax = efa.create_env(n_landm, [-2000, 8000], [-5000, 7000], 'rand')
        with open("env.pickle", "wb") as f:
            pickle.dump(env, f)
        with open("anchors.pickle", "wb") as f:
            pickle.dump(anchors, f)

    if save:
        path = efa.path_plan(0, 5000, path_samples, 'line', ax)
        with open("path.pickle", "wb") as f:
            pickle.dump(path, f)
        
    if save:
        obs, obs_anc, odom = efa.robot(env, anchors, path, 4000, 8000, True, ax)
        with open("obs.pickle", "wb") as f:
            pickle.dump(obs, f)
        with open("odom.pickle", "wb") as f:
            pickle.dump(odom, f)
        with open("obs_anc.pickle", "wb") as f:
            pickle.dump(obs_anc, f)
  
    plt.axis('equal')

    plt.grid(linestyle="--", color='black', alpha=0.3)
    plt.show()
    
    with open("env.pickle", "rb") as f1:
        env = pickle.load(f1)
    with open("path.pickle", "rb") as f2:
        path = pickle.load(f2)
    with open("obs.pickle", "rb") as f3:
        obs = pickle.load(f3)
    with open("odom.pickle", "rb") as f4:
        odom = pickle.load(f4)
    with open("obs_anc.pickle", "rb") as f5:
        obs_anc = pickle.load(f5)
    with open("anchors.pickle", "rb") as f6:
        anchors = pickle.load(f6)

    # with open("env.pickle", "wb") as f:
    #     pickle.dump(env.tolist(), f)
    # with open("path.pickle", "wb") as f:
    #     pickle.dump(path.tolist(), f)
    # with open("obs.pickle", "wb") as f:
    #     pickle.dump(obs.tolist(), f)
    # with open("odom.pickle", "wb") as f:
    #     pickle.dump(odom.tolist(), f)
    # with open("obs_anc.pickle", "wb") as f:
    #     pickle.dump(obs_anc.tolist(), f)
    # with open("anchors.pickle", "wb") as f:
    #     pickle.dump(anchors.tolist(), f)
    
    io.savemat("env.mat", {"array": env})
    io.savemat("path.mat", {"array": path})
    io.savemat("obs.mat", {"array": obs})
    io.savemat("odom.mat", {"array": odom})
    io.savemat("obs_anc.mat", {"array": obs_anc})
    io.savemat("anchors.mat", {"array": anchors})

    lms = ax.scatter(env[0, :], env[1, :], s=100, c='r', marker='o', label='Landmarks')
    fig.canvas.draw() 
    ancs = ax.scatter(anchors[:, 0], anchors[:, 1], s=100, c='r', marker='^', label='Anchors')
    fig.canvas.draw() 
    paths = ax.plot(path[:, 0], path[:, 1], c='g', label='Path', linewidth=2)
    fig.canvas.draw() 
    odoms = ax.scatter(odom[:, 0], odom[:, 1], s=10, c='k', marker='o', label='Odometry')
    est, = ax.plot([], [],marker='o', c='m', linewidth=3, label='Estimated')
    fig.canvas.draw() 
    est_lms = ax.scatter([], [], s=30, c='b', marker='o', label='Estimated Landmarks')
    fig.canvas.draw()
    est_ancs = ax.scatter([], [], s=30, c='b', marker='^', label='Estimated Anchors')
    fig.canvas.draw()
    plt.legend()
    fig.canvas.flush_events()
    # time.sleep(5)
    
    # Plot the observations
    # efa.plot_obs(obs, path, ax)
    plot_obs = 0
    if plot_obs == True:
        obs_lines, = ax.plot([], [], marker='o', linestyle='-', color='g', linewidth = 1, label='Observation')
        obs_lines_ancs, = ax.plot([], [], marker='o', linestyle='-', color='m', linewidth = 1, label='Anchor Observation')
        n_odom = np.shape(path)[0]
        n_lm = np.shape(obs)[1]
        for i in range(n_odom):
            x = []
            y = []
            for k in range(n_lm):
                if obs[i, k, 2] != 0:
                    x.append(path[i, 0])
                    x.append(path[i, 0] + obs[i, k, 0]*math.cos(obs[i, k, 1] + path[i, 2]))
                    y.append(path[i, 1])
                    y.append(path[i, 1] + obs[i, k, 0]*math.sin(obs[i, k, 1] + path[i, 2]))
            obs_lines.set_xdata(x)
            obs_lines.set_ydata(y)
            fig.canvas.draw()
            fig.canvas.flush_events()
            # time.sleep(1)
            # Anchor observations
            n_anc = np.shape(obs_anc)[1]
            x = []
            y = []
            for k in range(n_anc):
                if obs_anc[i, k] != 0:
                    x.append(path[i, 0])
                    x.append(anchors[k, 0])
                    y.append(path[i, 1])
                    y.append(anchors[k, 1])
            obs_lines_ancs.set_xdata(x)
            obs_lines_ancs.set_ydata(y)
            fig.canvas.draw()
            fig.canvas.flush_events()

    ini_landm_var = 1e6
    ini_landm_var = 1e4
    exp_landmarks = n_landm + 50
    exp_anchors = 4

    # projection matrix
    F = np.block([np.eye(3), np.zeros([3, 2*exp_landmarks]), np.zeros([3, 2*exp_anchors])])
    
    ini_mu = np.zeros([3 + 2*exp_landmarks + 2*exp_anchors, 1])

    ini_cov_xx = np.zeros([3, 3])
    ini_cov_xm = np.zeros([3, 2*exp_landmarks])
    ini_cov_mx = np.zeros([2*exp_landmarks, 3])
    ini_cov_mm = ini_landm_var * np.eye(2*exp_landmarks)
    ini_cov_xa = np.zeros([3, 2*exp_anchors])
    ini_cov_ax = np.zeros([2*exp_anchors, 3])
    ini_cov_ma = np.zeros([2*exp_landmarks, 2*exp_anchors])
    ini_cov_am = np.zeros([2*exp_anchors, 2*exp_landmarks])
    ini_cov_aa = np.eye(2*exp_anchors)*ini_landm_var
    ini_cov = np.block([[ini_cov_xx, ini_cov_xm, ini_cov_xa],
                        [ini_cov_mx, ini_cov_mm, ini_cov_ma],
                        [ini_cov_ax, ini_cov_am, ini_cov_aa]])

    # initial state
    sig = ini_cov
    mu = ini_mu

    # process noise
    R = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 0.0001]])
    
    # measurement noise
    Q = np.array([[22500, 0],
                  [0, 0.25]])
    
    # anchor noise
    Q_a = 100**2
    
    N = 0
    trig_initialized = [0, 0, 0, 0]
    ini_anc_pos = [-1500, 4000, 8500, 3500, 6500, -2000, -3000, -1500]
    ini_anc_pos = np.array([-2000, 4000, 8000, 3000, 6000, -2500, -2500, -1000]) #real_ancs
    ini_anc_pos = np.array([-2000, 4000, 8000, 3000, 3000, -2000, -2500, -1000]) #real_ancs

    # for i in range(1, path_samples):
    for i in range(1, 1000):
        print('i:', i)
        u = eka.odom2u(odom[i -1], odom[i])

        mu_bar, sig_bar = eka.ekf_unkown_predict(mu, sig, u, R, F)
        
        mu, sig, N = eka.ekf_unknown_correction(mu_bar, sig_bar, obs[i][:][:], Q, exp_landmarks, exp_anchors, N, 1)

        mu, sig, trig_initialized = eka.ekf_known_correction_anc(obs_anc[i, :], mu, sig, exp_landmarks, exp_anchors, Q_a, trig_initialized, ini_anc_pos)
        # print('mu:', mu[143:151])
        # if N > 4:
        #     pdb.set_trace()
        est.set_xdata(mu[0])
        est.set_ydata(mu[1])

        est_lms.set_offsets(np.column_stack((mu[3:3+2*N:2], mu[4:4+2*N:2])))
        fig.canvas.draw() 
        est_ancs.set_offsets(np.column_stack((mu[3 + 2*exp_landmarks:3+2*exp_landmarks+2*exp_anchors:2], mu[4 + 2*exp_landmarks:3+2*exp_landmarks+2*exp_anchors:2])))
        fig.canvas.draw() 
        fig.canvas.flush_events()

        # time.sleep(0.001)
        print('N:', N)
        # if N > 21:
        #     time.sleep(1000)
        # tmp = input('Press Enter to continue...')
    # time.sleep(1000)

if __name__ == '__main__':
    main()
    
