import math
import numpy as np
import matplotlib.pyplot as plt
import time

def odom2u(odom0, odom1):
    # odom0: [x0, y0, th0]
    # odom1: [x1, y1, th1]

    x0 = odom0[0]
    y0 = odom0[1]
    th0 = odom0[2]

    x1 = odom1[0]
    y1 = odom1[1]
    th1 = odom1[2]

    # Control input
    rot1 = math.atan2(y1 - y0, x1 - x0) - th0
    tran = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
    rot2 = th1 - th0 - rot1

    return [tran, rot1, rot2]

def ekf_unkown_predict(mu, sig, u, R, F):
    # mu: mean of the state
    # sig: covariance of the state
    # u: control input
    # R: control noise

    # Get the control input
    tran = u[0]
    rot1 = u[1]
    rot2 = u[2]

    # Odometry model
    odo = np.array([[tran*math.cos(rot1 + mu[2])],
            [tran*math.sin(rot1 + mu[2])],
            [rot1 + rot2]])
    
    # Predicted state
    mu_bar = mu + np.matmul(np.transpose(F), odo)
    
    # Jacobian of the motion model
    g = np.array([[0, 0, -tran*math.sin(rot1 + mu[2])],
            [0, 0,  tran*math.cos(rot1 + mu[2])],
            [0, 0, 0]])
    
    G = np.eye(len(mu)) + np.matmul(np.matmul(np.transpose(F), g), F)

    # Predicted covariance
    sig_bar = np.matmul(np.matmul(G, sig), np.transpose(G)) + np.matmul(np.matmul(F.T, R), F)

    return mu_bar, sig_bar

def ekf_unknown_obs_correction(k, mu_bar, sig_bar, exp_landmarks, z, Q):
    # mu_bar: predicted mean of the state
    # sig_bar: predicted covariance of the state
    # z: measurement
    # Q: measurement noise
    # k: index of the landmark

    # dell: difference between the landmark and the robot
    dell = np.concatenate((mu_bar[3 + 2*k] - mu_bar[0], mu_bar[4 + 2*k] - mu_bar[1]), axis=0)
    q = np.matmul(dell.T, dell)
    sqrt_q = math.sqrt(q)

    # Expected measurement
    z0 = np.array([[sqrt_q], math.atan2(dell[1], dell[0]) - mu_bar[2]])

    # F_xk: Projection matrix
    F_xk = np.concatenate((np.concatenate((np.eye(3), np.zeros([3, 2*k]), np.zeros([3, 2]), np.zeros([3, 2*exp_landmarks - 2*k - 2])), axis =1),
                           np.concatenate((np.zeros([2, 3 + 2*k]), np.eye(2), np.zeros([2, 2*(exp_landmarks - k - 1)])), axis =1)), axis =0)

    # H: Jacobian of the measurement model
    H0 = 1/q * np.block([[-sqrt_q*dell[0], -sqrt_q*dell[1], 0, sqrt_q*dell[0], sqrt_q*dell[1]],
                        [dell[1], -dell[0], -q, -dell[1], dell[0]]])
    
    H = np.matmul(H0, F_xk)

    # Innovation
    psi = np.matmul(np.matmul(H, sig_bar), np.transpose(H)) + Q

    del_z = z - z0
    if del_z[1] > 5.5:
        print("z:", z)
        print("z0:", z0)
        print("del_z[1]:", del_z[1])
        del_z[1] = 2*math.pi - del_z[1]
        print('**********')
        time.sleep(10)

    if del_z[1] < -5.5:
        print("z:", z)
        print("z0:", z0)
        print("del_z[1]:", del_z[1])
        del_z[1] = del_z[1] + 2*math.pi
        print('##########')
        time.sleep(10)

    pie = np.matmul(np.matmul(np.transpose(z - z0), np.linalg.inv(psi)), (z - z0))

    return pie, psi, H, z0

def ekf_unknown_correction(mu_bar, sig_bar, obs, Q, exp_N_landm, N, alp):
    # mu_bar: predicted mean of the state
    # sig_bar: predicted covariance of the state

    # H: Jacobian of the measurement model
    # z0: expected measurement
    
    obs = np.squeeze(obs)
    # print('obs:', obs)
    len_obs = len(obs)

    for j in range(len_obs):
        if obs[j,2] != 0:
            # Measurement
            r = obs[j,0]
            phi = obs[j,1]

            z = np.array([[r], [phi]])

            # Add the landmark to the state
            rel_meas = np.array([[r*math.cos(phi + mu_bar[2])], [r*math.sin(phi + mu_bar[2])]])

            mu_bar[1 + 2*(N +1): 3 + 2*(N +1)] = mu_bar[0:2] + rel_meas

            pie = np.zeros(N +1)

            # Compute the innovation for each landmark
            for k in range(N + 1):
                pie[k], _, _, _ = ekf_unknown_obs_correction(k, mu_bar, sig_bar, exp_N_landm, z, Q)
            
            # Add a new landmark
            pie[N] = alp
            print('pie:', pie)
            ind_j = np.argmin(pie)

            # If the measurement is not associated with any landmark, add a new landmark
            # Based on Mahalanobis distance
            _, psi_j, H_j, z_j = ekf_unknown_obs_correction(ind_j, mu_bar, sig_bar, exp_N_landm, z, Q)
            N = max(N, ind_j +1)

            # Find the Kalman gain
            K = np.matmul(np.matmul(sig_bar, np.transpose(H_j)), np.linalg.inv(psi_j))

            mu_bar = mu_bar + np.matmul(K, (z - z_j))

            sig_bar = np.matmul((np.eye(len(mu_bar)) - np.matmul(K, H_j)), sig_bar)
    
    mu = mu_bar
    sig = sig_bar

    return mu, sig, N