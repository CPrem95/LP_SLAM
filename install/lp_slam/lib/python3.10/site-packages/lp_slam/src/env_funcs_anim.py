import math
import numpy as np
import random
import matplotlib.pyplot as plt
import pickle
import time

def create_env(n, limx, limy, env_type):
    
    # Create random x, y coordinates
    if env_type == 'rand':
        x = [random.randint(limx[0], limx[1]) for _ in range(n)]
        y = [random.randint(limy[0], limy[1]) for _ in range(n)]

    env = np.array([x, y])

    anchors = np.array([
        [-2000, 4000],
        [8000, 3000],
        #[6000, -2500],
        [3000, -2000],
        [-2500, -1000]
    ])

    # Plot the data with equal axes
    fig, ax = plt.subplots()
    ax.scatter(x, y, marker='o', color='b', label='Objects')
    ax.scatter(anchors[:, 0], anchors[:, 1],  marker='^', color='b', label='Anchors')

    # Add labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('X-Y Plot')

    # Set equal axes
    plt.axis('equal')

    # Show the legend
    plt.legend()

    # Display the plot
    # plt.show()

    return env, anchors, ax

def path_plan(start, stop, n, path_type, ax):
    th = np.zeros(n)
    if path_type == 'simple':
        x = np.linspace(start, stop, n)
        y = np.exp(1.5*(x/1000))

        
    if path_type == 'line':
        x = np.linspace(start, stop, n)
        y = 0.1*x + 0.001
        

    for i in range(1,n):
        th[i] = math.atan2(y[i] - y[i -1], x[i] - x[i -1])

    x = x[0:n]
    y = y[0:n]
    th = th[0:n]

    x[0] = 0
    y[0] = 0
    th[0] = 0

    true_path = np.transpose([x, y, th])
    nSamples = true_path.shape[0]

    # Plot the data with equal axes
    ax.plot(x, y, marker='o', linestyle='-', color='b', label='True path')

    # Add labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('X-Y Plot')

    # Set equal axes
    plt.axis('equal')

    # Show the legend
    plt.legend()

    # Display the plot
    # plt.show()
    return true_path


def robot(env, anchors, path, r_thresh_obs, r_thresh_anc, vis, ax):
    n_odom = np.shape(path)[0]
    n_anc = np.shape(anchors)[0]

    rob_r = 0 # robot radius = 250
    odom = path.copy()

    n_lm = np.shape(env)[1]
    obs = np.zeros((n_odom, n_lm, 3))
    obs_anc = np.zeros((n_odom, n_anc))

    sum_err_x = 0
    sum_err_y = 0
    sum_err_th = 0
    mul1 = 1 # odometry error
    mul2 = 0 # point error
    mul3 = 1 # anchor error
    for i in range(2, n_odom):
        sum_err_x = sum_err_x + mul1*random.normalvariate(1, 0.005)
        sum_err_y = sum_err_y - mul1*random.normalvariate(0.5, 0.02)
        sum_err_th = sum_err_th + mul1*random.normalvariate(0, 0.01)

        odom[i, 0] = odom[i, 0] + sum_err_x
        odom[i, 1] = odom[i, 1] + sum_err_y
        odom[i, 2] = odom[i, 2] + sum_err_th

    # Observations
    for i in range(n_odom):
        for k in range(n_lm):
            dx = env[0, k] - path[i, 0]
            dy = env[1, k] - path[i, 1]

            r = math.sqrt(dx**2 + dy**2)
            th = math.atan2(dy, dx) - path[i, 2]

            if r < r_thresh_obs and False:
                obs[i, k, 0] = r + mul2*random.normalvariate(0, 0.1)
                obs[i, k, 1] = th + mul2*random.normalvariate(0, 0.03)
                obs[i, k, 2] = k +1 # to avoid first zero index and to keep track of the landmarks
    # Anchor observations
    for i in range(n_odom):
        for k in range(n_anc):
            dx = anchors[k, 0] - (path[i, 0] + rob_r*math.cos(path[i, 2]))
            dy = anchors[k, 1] - (path[i, 1] + rob_r*math.sin(path[i, 2]))

            r = math.sqrt(dx**2 + dy**2)

            if r < r_thresh_anc:
                obs_anc[i, k] = r + mul3*random.normalvariate(0, 100)
        
    if vis == True:
        # Plot the data with equal axes
        ax.plot(odom[:, 0], odom[:, 1], marker='o', linestyle='-', color='r', label='Odometry')

        # Add labels and title
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('X-Y Plot')

        # Set equal axes
        plt.axis('equal')

        # Show the legend
        plt.legend()

        # Display the plot
        # plt.show()
    
    return obs, obs_anc, odom

def plot_obs(obs, path, ax):
    n_odom = np.shape(path)[0]
    n_lm = np.shape(obs)[1]
    # Plot the data with equal axes

    for i in range(n_odom):
        for k in range(n_lm):
            if obs[i, k, 2] != 0:
                x = path[i, 0] + obs[i, k, 0]*math.cos(obs[i, k, 1] + path[i, 2])
                y = path[i, 1] + obs[i, k, 0]*math.sin(obs[i, k, 1] + path[i, 2])

                ax.plot([path[i, 0], x], [path[i, 1], y], marker='o', linestyle='-', color='g', label='Observation')

                print('points: ', path[i, 0], x, path[i, 1], y)
        time.sleep(1)