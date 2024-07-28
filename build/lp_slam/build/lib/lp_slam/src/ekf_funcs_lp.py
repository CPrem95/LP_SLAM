# pylint: disable=C0103, C0116, W0611, C0302, C0301, C0303, C0114
import math
import time
import pdb
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

# Odometry to control input
def odom2u(odom0, odom1):
    """ Convert odometry to control input."""
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

# Test continuity count statistic
def cont_count(bins, thresh):
    """ Test continuity count statistic."""
    n = len(bins)
    stat = sum(bins >= thresh) / n
    return stat

# Find maximum Q value
def find_maxQ(n_bins, N_bar):
    """ Find maximum Q value stat."""
    maxQ = N_bar * (n_bins - 1) + N_bar * (n_bins - 1) ** 2
    return maxQ

# Find Quadrat count statistic:: main function
def quadratC(inlierPts, x, y, m, fig):
    """ Find Quadrat count statistic."""
    # inlierPts: inlier points
    # x: x values of the line
    # y: y values of the line
    # m: number of quadrats
    # fig: plot the points, line, and quadrat midpoints
    
    if m == 0: # When line segment is less than the quadrat size, m can be zero
        m = 1  # set m to 1 to avoid division by zero, and to consider the whole line segment

    n_Pts = inlierPts.shape[0]  # number of points
    bins = np.zeros(m)  # initialize bins

    x_min, x_max = x[0], x[1]  # minimum and maximum x values of the line
    y_min, y_max = y[0], y[1]  # minimum and maximum y values of the line

    X_quad = np.linspace(x_min, x_max, 2 * m + 1)  # x values for quadrat midpoints
    Y_quad = np.linspace(y_min, y_max, 2 * m + 1)  # y values for quadrat midpoints

    X_quad_mid = X_quad[1::2]  # x values for quadrat midpoints excluding boundaries
    Y_quad_mid = Y_quad[1::2]  # y values for quadrat midpoints excluding boundaries
    quadMid = np.column_stack((X_quad_mid, Y_quad_mid))  # coordinates of quadrat midpoints

    for i in range(n_Pts):  # iterate over each point
        dist = np.zeros(m)  # initialize distance array
        for j in range(m):  # iterate over each quadrat midpoint
            P = np.vstack((inlierPts[i], quadMid[j]))  # create a line segment between the point and quadrat midpoint
            dist[j] = np.linalg.norm(P[0] - P[1])  # calculate the distance between the point and quadrat midpoint
        I = np.argmin(dist)  # find the index of the closest quadrat midpoint
        bins[I] += 1  # increment the count of the corresponding bin

    N_bar = n_Pts / m  # expected number of points per bin
    maxQ = find_maxQ(m, N_bar)  # maximum Q value
    thresh = 1  # threshold for continuity count
    stat = cont_count(bins, thresh)  # continuity count statistic
    Q = np.sum((bins - N_bar) ** 2) / N_bar  # Quadrat count statistic
    mod_bins = np.copy(bins)  # create a copy of bins
    mod_bins[mod_bins > N_bar] = N_bar  # limit the count of each bin to N_bar
    modQ = np.sum((mod_bins - N_bar) ** 2) / N_bar  # modified Quadrat count statistic

    if fig:  # if fig is True, plot the points, line, and quadrat midpoints
        plt.scatter(inlierPts[:, 0], inlierPts[:, 1], color='r')
        plt.plot(x, y, 'g-')
        plt.scatter(X_quad_mid, Y_quad_mid, marker="*")
        plt.show()

    return Q, modQ, maxQ, n_Pts, bins, stat, N_bar


# Find Quadrat count statistic:: secondary function
def quadratC2(bins):
    n_Pts = sum(bins) # number of points
    n_bins = len(bins) # number of bins

    N_bar = n_Pts / n_bins # expected number of points per bin

    maxQ = find_maxQ(n_bins, N_bar)  # maximum Q value

    thresh = 1  # threshold for continuity count
    stat = cont_count(bins, thresh)  # continuity count statistic

    Q = sum((bins - N_bar) ** 2) / N_bar  # Quadrat count statistic
    bins[bins > N_bar] = N_bar  # limit the count of each bin to N_bar
    modQ = sum((bins - N_bar) ** 2) / N_bar  # modified Quadrat count statistic

    return Q, modQ, maxQ, n_Pts, bins, stat, N_bar

# Find the consecutive elements in an array
def findConsec(arr, val):
    # Initialize variables
    consecutive_count = 0 # count of consecutive elements
    consecutive_elements = np.array([]) # consecutive elements
    max_count = 0 # maximum count of consecutive elements
    arr_L = len(arr) # length of the array

    # Loop through the array
    for i in range(arr_L):
        if arr[i] >= val:
            consecutive_count += 1
            consecutive_elements = np.append(consecutive_elements, arr[i])
            if consecutive_count > max_count:
                max_count = consecutive_count # update the maximum count
                max_cons_elements = np.copy(consecutive_elements)# update the maximum consecutive elements
                max_ind = i
        else:
            # Reset consecutive elements and count
            consecutive_count = 0
            consecutive_elements = np.array([])

    if False: # Whether to apply tail ends correction
        # Correction for tail ends observations: e.g. [2, 0, 1, 4, 5, 1, 9, 0, 6, 0, 7, 8]
        ind1 = max_ind - max_count - 1 # index of the element before the start of the consecutive elements
        ind2 = max_ind + 2 # index of the element after the end of the consecutive elements
        if ind1 > 0:
            if arr[ind1] > val:
                max_count += 2
                np.concatenate((max_cons_elements, arr[ind1: ind1 + 2]))
        if ind2 < len(arr):
            if arr[ind2] > val:
                max_count += 2
                np.concatenate((max_cons_elements, arr[ind2 - 1: ind2]))

    return max_count, max_cons_elements

# Dind perpendicular distance between a point and a line
def pt2line(lineParams, point):
    # lineParams = [m, c]
    m = lineParams[0]
    c = lineParams[1]

    x = point[0]
    y = point[1]

    D = abs(m * x - y + c) / math.sqrt(m ** 2 + 1)
    return D

# Convert m and c to line parameters
def mc2line(X, points, randPts, m, c, figID, isFig):
    isFig = False
    
    if isFig:
        X = [min(randPts[0]), max(randPts[0])]
        Y = [m * X[0] + c, m * X[1] + c]

        plt.figure(figID)
        plt.figure(figID).clear()
        plt.scatter(points[:, 0], points[:, 1], color='black')
        plt.scatter(randPts[0], randPts[1], color='red', marker='o', edgecolors='black')
        plt.plot(X, Y, color='blue', linewidth=2)
        plt.xlim(X)
        plt.ylim([-2500, 2500])
        plt.ylim([min(points[:, 1]), max(points[:, 1])])
        plt.axis('equal')
        plt.pause(0.1)
        plt.show()

#___________________________________________________________________________________________________________________________
#
#Line Extraction
#___________________________________________________________________________________________________________________________

# Find the linear regression model
def lin_regress(x, y):
    # Simple linear regression
    # x = x coordinates of the points
    # y = y coordinates of the points
    
    # Calculate the mean of x and y coordinates
    x_bar = np.mean(x)
    y_bar = np.mean(y)

    # Calculate the deviations from the mean
    x1 = x - x_bar
    y1 = y - y_bar

    # Calculate the cross product and squared deviations
    xy = x1 * y1
    x_sq = x1 ** 2

    # Calculate the slope and intercept of the line
    m = np.sum(xy) / np.sum(x_sq)
    c = y_bar - m * x_bar

    # Create the model as a list of slope and intercept
    model = [m, c]

    # Return the model
    return model

# Find the length of the line
def findLineLen(inliers):
    # function to find the length of the line having all the inliers.

    x = [np.min(inliers[:, 0]), np.max(inliers[:, 0])]
    y = [np.min(inliers[:, 1]), np.max(inliers[:, 1])]

    lineLen = np.sqrt((x[0] - x[1])**2 + (y[0] - y[1])**2)
    return lineLen

# Vanilla RANSAC algorithm
def myRANSAC2(points, sampleSize, maxDistance):
    # points: coordinates of the points
    # sampleSize: number of points to sample per trial
    # maxDistance: max allowable distance for inliers   

    modelR = True  # Flag to indicate if a model was found
    Trials = 30  # Number of RANSAC trials
    MaxInliers = sampleSize  # initialization value for maximum number of inliers
    WinningLineParams = [0, 0]  # initialization value for winning line parameters (m and c)
    # inliers_mem = np.zeros(Trials)  # array to store the number of inliers for each trial
    # minQ_mem = np.zeros(Trials)
    # minStat_mem = np.zeros(Trials) 

    nPts = points.shape[0]  # number of points
    if nPts < sampleSize:
        print('ERROR:: More samples than number of points!!!')
        return None

    inlierIdx = np.zeros(nPts, dtype=bool)  # array to store the indices of inliers
    GoodQ = 0  # initialization value for GoodQ
    GoodStat = 0  # initialization value for GoodStat
    Xlim = [-1000, 4000]  # x-axis limits for plotting
    Xlim = [-2000, 2000]  # updated x-axis limits for plotting

    for iter in range(Trials):
        tmp_InlierIdx = np.zeros(nPts, dtype=bool)  # temporary array to store the indices of inliers for the current trial
        CurrentInliers = np.zeros((0, 2))  # temporary array to store the coordinates of inliers for the current trial
        inliers = 0  # counter for the number of inliers
        rand_perm = np.random.permutation(nPts)  # random permutation of indices
        rand_num = rand_perm[:sampleSize]  # randomly selected indices for sampling

        # x and y coordinates of random points
        px = points[rand_num, 0]
        py = points[rand_num, 1]

        # start_pt = [np.min(points[:, 0]), np.min(points[:, 1])]
        # end_pt = [np.max(points[:, 0]), np.max(points[:, 1])]
        # px = [start_pt[0], end_pt[0]]
        # py = [start_pt[1], end_pt[1]]

        # fitting the random points to a regression line
        fitLine = lin_regress(px, py)
        m = fitLine[0]
        c = fitLine[1]

        # find the distance from every point to the line
        for counter in range(nPts):
            d = pt2line([m, c], points[counter, :])
            # label the point an inlier if its distance to the line is below the threshold
            if d < maxDistance:
                inliers += 1
                tmp_InlierIdx[counter] = True
                CurrentInliers = np.vstack((points[counter], CurrentInliers))

        tmp_inliers = inliers
        terminate = True
        loopCount = 1
        inl_array = []
        max_inl = 10000

        # Executes until a proper line is found
        while terminate:  # Loop until termination condition is met
            if inliers > 5:  # Check if there are enough inliers
                fitLine = lin_regress(points[tmp_InlierIdx, 0], points[tmp_InlierIdx, 1])  # Fit a line to the inliers
                m = fitLine[0]  # Slope of the line
                c = fitLine[1]  # Intercept of the line

                tmp_InlierIdx = np.zeros(nPts, dtype=bool)  # Temporary array to store the indices of inliers
                CurrentInliers = np.zeros((0, 2))  # Temporary array to store the coordinates of inliers
                inliers = 0  # Counter for the number of inliers

                for counter in range(nPts):  # Iterate through all points
                    d = pt2line([m, c], points[counter, :])  # Calculate the distance from the point to the line
                    if d < maxDistance:  # Check if the point is an inlier
                        inliers += 1  # Increment the inlier count
                        tmp_InlierIdx[counter] = True  # Mark the point as an inlier
                        CurrentInliers = np.vstack((points[counter], CurrentInliers))  # Add the point to the inliers array
                loopCount += 1  # Increment the loop count

            if tmp_inliers == inliers or max_inl == inliers:  # Check termination conditions
                # print('Inliers:', inliers)  # Print the number of inliers
                terminate = False  # Terminate the loop
            elif loopCount > 20:  # Check if termination condition is met
                # print('Looping!!!  ', loopCount)
                inl_array.append(inliers)  # Store the number of inliers
                if loopCount > 25:  # Check if enough values have been stored
                    max_inl = max(inl_array)  # Find the maximum number of inliers
                    # print('adjusted max_inl: ', max_inl)  # Print the maximum number of inliers
                    if loopCount > 30:  # Check if termination condition is met, sometimes the line still tends to oscillate
                        terminate = False  # Terminate the loop

            # print('inliers, tmp_inliers: ', inliers, tmp_inliers)  # Print the number of inliers for the current and previous trial
            tmp_inliers = inliers  # Update the temporary inlier count

        # inliers_mem[iter] = inliers  # Store the number of inliers for this trial
        
        # After finding a proper local line, tries to find the best global line
        if inliers > MaxInliers:  # Check if the current model has more inliers than the previous best model
            modelInliers = lin_regress(points[tmp_InlierIdx, 0], points[tmp_InlierIdx, 1])  # Fit a line to the inliers
            m = modelInliers[0]  # Slope of the line
            c = modelInliers[1]  # Intercept of the line

            inlierIdx = tmp_InlierIdx  # Update the indices of the inliers
            WinningLineParams = [m, c]  # Update the parameters of the winning line
            MaxInliers = inliers  # Update the maximum number of inliers
            # print('**Line found:', [inliers, lineLength, stat, minQ, Q])  # Print the number of inliers, continuity count statistic, minimum Q value, and Q value
            # GoodQ = Q  # Update the good Q value
            # GoodStat = stat  # Update the good statistic value

    if WinningLineParams == [0, 0]:  # Check if a model was found
        # print('L_Find completed! no lines!!')  # Print a message indicating that line finding is completed
        inlierIdx = tmp_InlierIdx  # Update the indices of the inliers
        modelR = False  # Set the model flag to False
    # else:
        # print('inliers_mem: ', inliers_mem) # Print the number of inliers for each trial
        # print('minQ_mem: ', minQ_mem)  # Print the minimum Q value for each trial
        # print('minStat_mem: ', minStat_mem)
        # pass
        
    mc2line(Xlim, points, [points[inlierIdx, 0], points[inlierIdx, 1]], WinningLineParams[0], WinningLineParams[1], 1000, True)  # Plot the line
    # time.sleep(0.01)  # Pause for visualization
    
    return modelR, inlierIdx, WinningLineParams  # Return the model flag, indices of the inliers, and parameters of the winning line
# modified RANSAC algorithm
def myRANSAC(points, sampleSize, maxDistance, L_thresh_n):
    # points: coordinates of the points
    # sampleSize: number of points to sample per trial
    # maxDistance: max allowable distance for inliers   

    modelR = True  # Flag to indicate if a model was found
    Trials = 30  # Number of RANSAC trials
    MaxInliers = sampleSize  # initialization value for maximum number of inliers
    WinningLineParams = [0, 0]  # initialization value for winning line parameters (m and c)
    # inliers_mem = np.zeros(Trials)  # array to store the number of inliers for each trial
    # minQ_mem = np.zeros(Trials)
    # minStat_mem = np.zeros(Trials) 

    nPts = points.shape[0]  # number of points
    if nPts < sampleSize:
        print('ERROR:: More samples than number of points!!!')
        return None

    inlierIdx = np.zeros(nPts, dtype=bool)  # array to store the indices of inliers
    GoodQ = 0  # initialization value for GoodQ
    GoodStat = 0  # initialization value for GoodStat
    Xlim = [-1000, 4000]  # x-axis limits for plotting
    Xlim = [-2000, 2000]  # updated x-axis limits for plotting

    for iter in range(Trials):
        tmp_InlierIdx = np.zeros(nPts, dtype=bool)  # temporary array to store the indices of inliers for the current trial
        CurrentInliers = np.zeros((0, 2))  # temporary array to store the coordinates of inliers for the current trial
        inliers = 0  # counter for the number of inliers
        rand_perm = np.random.permutation(nPts)  # random permutation of indices
        rand_num = rand_perm[:sampleSize]  # randomly selected indices for sampling

        # x and y coordinates of random points
        px = points[rand_num, 0]
        py = points[rand_num, 1]

        # start_pt = [np.min(points[:, 0]), np.min(points[:, 1])]
        # end_pt = [np.max(points[:, 0]), np.max(points[:, 1])]
        # px = [start_pt[0], end_pt[0]]
        # py = [start_pt[1], end_pt[1]]

        # fitting the random points to a regression line
        fitLine = lin_regress(px, py)
        m = fitLine[0]
        c = fitLine[1]

        # find the distance from every point to the line
        for counter in range(nPts):
            d = pt2line([m, c], points[counter, :])
            # label the point an inlier if its distance to the line is below the threshold
            if d < maxDistance:
                inliers += 1
                tmp_InlierIdx[counter] = True
                CurrentInliers = np.vstack((points[counter], CurrentInliers))

        tmp_inliers = inliers
        terminate = True
        loopCount = 1
        inl_array = []
        max_inl = 10000

        # Executes until a proper line is found
        while terminate:  # Loop until termination condition is met
            if inliers > 5:  # Check if there are enough inliers
                fitLine = lin_regress(points[tmp_InlierIdx, 0], points[tmp_InlierIdx, 1])  # Fit a line to the inliers
                m = fitLine[0]  # Slope of the line
                c = fitLine[1]  # Intercept of the line

                tmp_InlierIdx = np.zeros(nPts, dtype=bool)  # Temporary array to store the indices of inliers
                CurrentInliers = np.zeros((0, 2))  # Temporary array to store the coordinates of inliers
                inliers = 0  # Counter for the number of inliers

                for counter in range(nPts):  # Iterate through all points
                    d = pt2line([m, c], points[counter, :])  # Calculate the distance from the point to the line
                    if d < maxDistance:  # Check if the point is an inlier
                        inliers += 1  # Increment the inlier count
                        tmp_InlierIdx[counter] = True  # Mark the point as an inlier
                        CurrentInliers = np.vstack((points[counter], CurrentInliers))  # Add the point to the inliers array
                loopCount += 1  # Increment the loop count

            if tmp_inliers == inliers or max_inl == inliers:  # Check termination conditions
                # print('Inliers:', inliers)  # Print the number of inliers
                terminate = False  # Terminate the loop
            elif loopCount > 20:  # Check if termination condition is met
                # print('Looping!!!  ', loopCount)
                inl_array.append(inliers)  # Store the number of inliers
                if loopCount > 25:  # Check if enough values have been stored
                    max_inl = max(inl_array)  # Find the maximum number of inliers
                    # print('adjusted max_inl: ', max_inl)  # Print the maximum number of inliers
                    if loopCount > 30:  # Check if termination condition is met, sometimes the line still tends to oscillate
                        terminate = False  # Terminate the loop

            # print('inliers, tmp_inliers: ', inliers, tmp_inliers)  # Print the number of inliers for the current and previous trial
            tmp_inliers = inliers  # Update the temporary inlier count

        # inliers_mem[iter] = inliers  # Store the number of inliers for this trial
        
        # After finding a proper local line, tries to find the best global line
        if inliers > MaxInliers:  # Check if the current model has more inliers than the previous best model
            modelInliers = lin_regress(points[tmp_InlierIdx, 0], points[tmp_InlierIdx, 1])  # Fit a line to the inliers
            m = modelInliers[0]  # Slope of the line
            c = modelInliers[1]  # Intercept of the line

            lineLength = findLineLen(CurrentInliers)  # Calculate the length of the line
            n_bins = round(lineLength / 50)  # Calculate the number of bins for the quadrat count statistic
            x = [np.min(CurrentInliers[:, 0]), np.max(CurrentInliers[:, 0])]  # x-coordinates of the line
            
            # y = x * m + c  # y-coordinates of the line
            y = np.multiply(x, m) + c
            Q, _, _, _, values, stat, mu = quadratC(CurrentInliers, x, y, n_bins, False)  # Calculate the quadrat count statistic

            minQ = n_bins * mu  # Calculate the minimum Q value
            scaled_minQ = minQ*1.5  # Calculate the scaled minimum Q value

            if scaled_minQ < Q and len(values) > 5:  # Check if the minimum Q value is less than the calculated Q value
                max_count, max_cons_elements = findConsec(values, 1)  # Find the consecutive elements in the values array
                if max_count >= (L_thresh_n - 2):  # Check if there are enough consecutive elements
                    # pdb.set_trace()
                    Q, _, _, _, _, stat, _ = quadratC2(max_cons_elements)  # Calculate the quadrat count statistic for the consecutive elements
                    minQ = max_count * mu  # Calculate the minimum Q value
                    scaled_minQ = minQ*1.5  # Calculate the scaled minimum Q value

            # minQ_mem[iter] = Q  # Store the minimum Q value
            # minStat_mem[iter] = minQ  # Store the continuity count statistic
            # print('$$Line:', [inliers, lineLength, stat, minQ, Q])
            if scaled_minQ > Q and stat > 0.8:  # Check if the minimum Q value is greater than the calculated Q value and the statistic is above the threshold
                inlierIdx = tmp_InlierIdx  # Update the indices of the inliers
                WinningLineParams = [m, c]  # Update the parameters of the winning line
                MaxInliers = inliers  # Update the maximum number of inliers
                # print('**Line found:', [inliers, lineLength, stat, minQ, Q])  # Print the number of inliers, continuity count statistic, minimum Q value, and Q value
                # GoodQ = Q  # Update the good Q value
                # GoodStat = stat  # Update the good statistic value

    if WinningLineParams == [0, 0]:  # Check if a model was found
        # print('L_Find completed! no lines!!')  # Print a message indicating that line finding is completed
        inlierIdx = tmp_InlierIdx  # Update the indices of the inliers
        modelR = False  # Set the model flag to False
    # else:
        # print('inliers_mem: ', inliers_mem) # Print the number of inliers for each trial
        # print('minQ_mem: ', minQ_mem)  # Print the minimum Q value for each trial
        # print('minStat_mem: ', minStat_mem)
        # pass
        
    mc2line(Xlim, points, [points[inlierIdx, 0], points[inlierIdx, 1]], WinningLineParams[0], WinningLineParams[1], 1000, True)  # Plot the line
    # time.sleep(0.01)  # Pause for visualization
    
    return modelR, inlierIdx, WinningLineParams  # Return the model flag, indices of the inliers, and parameters of the winning line

# To check intersection of the line with the robot. read paper for more details
def chkIntersec(mu, ro, alp):
    # Calculate the slope and y-intercept of the observed line
    m1, c1 = roalp2mc(mu, ro, alp)

    # Calculate the slope and y-intercept of the line connecting origin to robot
    m2 = mu[1] / mu[0]
    c2 = 0

    # Calculate the intersection point of the two lines
    inter = [c1, m2 * c1] / (m2 - m1)

    # Check if the intersection point lies within the range of robot's x-coordinate
    if mu[0] > 0:
        isInter = (0 < inter[0]) and (inter[0] < mu[0])
    else:
        isInter = (0 > inter[0]) and (inter[0] > mu[0])

    return isInter

def chkIntersec2(mu, m1, c1):
    isInter = False

    m2 = mu[1] / mu[0]
    m2 = m2[0]
    c2 = 0  # origin to robot

    inter = [c1, m2 * c1] / (m2 - m1)

    if mu[0] > 0:
        if (0 < inter[0]) and (inter[0] < mu[0]):
            isInter = True
    else:
        if (0 > inter[0]) and (inter[0] > mu[0]):
            isInter = True

    return isInter

def chkIntersec3(mu, r, psi):
    # Calculate the slope and y-intercept of the observed line
    m1 = np.tan(np.pi / 2 + psi)
    c1 = r / np.sin(psi)

    # Calculate the slope and y-intercept of the line connecting origin to robot
    m2 = mu[1] / mu[0]
    c2 = 0

    # Calculate the intersection point of the two lines
    inter = np.array([c1, m2 * c1]) / (m2 - m1)

    # Check if the intersection point lies within the range of robot's x-coordinate
    if mu[0] > 0:
        isInter = (0 < inter[0]) and (inter[0] < mu[0])
    else:
        isInter = (0 > inter[0]) and (inter[0] > mu[0])

    return isInter

# To mitigate the effect of the slope of the line in linear regression when the slope is close to 90 degrees
def modify_lineParams(isLine, inlierIdx, line_params, thresh_th, points, sampleSize, maxDistance, L_thresh_n):
    m = line_params[0]
    m_th = np.degrees(np.arctan(m))  # converts the slope to an angle
    if isLine:
    # pdb.set_trace()
        if (-90 < m_th < -thresh_th) or (thresh_th < m_th < 90):
            mod_points = np.column_stack((points[:, 1], points[:, 0]))
            isLine, inlierIdx, model = myRANSAC(mod_points, sampleSize, maxDistance, L_thresh_n)
            if isLine:
                m_rot = model[0]
                line_params[0] = 1 / m_rot  # m
                line_params[1] = -model[1] / m_rot  # c

            # Xlim = [-2000, 2000]
            # mc2line(Xlim, points, [points[inlierIdx, 0], points[inlierIdx, 1]], line_params[0], line_params[1], 1000, True)
            # time.sleep(0.01)
    return isLine, inlierIdx, line_params

# Used in LineExtract to fit a line to the inliers usinf ransac
# Fit a line to the inliers using polyfit
def fitransac(pts, odoms, rob_obs_pose, L_thresh_n, D_ransac, fig):
    lineParams_robot = 0
    outlierPts = np.array([0])
    inlierPts = np.array([0])

    sampleSize = 3  # number of points to sample per trial
    # maxDistance = 15  # max allowable distance for inliers
    # maxDistance = D_ransac # max allowable distance for inliers

    # print("\nfitransac running")
    isLine, inlierIdx, modelInliers = myRANSAC(pts, sampleSize, D_ransac, L_thresh_n)

    isLine, inlierIdx, modelInliers = modify_lineParams(isLine, inlierIdx, modelInliers, 70, pts, sampleSize, D_ransac, L_thresh_n)

    if isLine:
        # Inliers
        inlierPts = pts[inlierIdx, :]

        # Outliers
        outlierPts = pts[~inlierIdx, :]

        # Refit a line to the inliers using polyfit.
        # Line parameters::: y = mx + c
        m = modelInliers[0]
        c = modelInliers[1]

        x0 = -c * m / (m ** 2 + 1)
        y0 = -x0 / m

        # Line start/end points
        x = [np.min(inlierPts[:, 0]), np.max(inlierPts[:, 0])]
        if m > 0:
            y = [np.min(inlierPts[:, 1]), np.max(inlierPts[:, 1])]
        else:
            y = [np.max(inlierPts[:, 1]), np.min(inlierPts[:, 1])]

        # Line params from the origin
        r = np.sqrt(x0 ** 2 + y0 ** 2)
        th = np.arctan2(y0, x0)
        len_line = np.sqrt((x[0] - x[1]) ** 2 + (y[0] - y[1]) ** 2)

        # Line params as seen from the robot
        odompt = rob_obs_pose
        isInter = chkIntersec2(rob_obs_pose, m, c)

        if isInter:
            r0 = -r + odompt[0] * np.cos(th) + odompt[1] * np.sin(th)
            th0 = th - odompt[2] + np.pi
        else:
            r0 = r - odompt[0] * np.cos(th) - odompt[1] * np.sin(th)
            th0 = th - odompt[2]

        lineParams_robot = [r0.item(), th0.item(), len_line, m, c, x[0], x[1], y[0], y[1]] # line parameters as seen from the robot [r0, th0, len, m, c, x0, x1, y0, y1]

        # Plotting
        if fig:
            plt.figure()

            if m < 0:
                x.reverse()

            # show inliers and outliers
            plt.scatter(inlierPts[:, 0], inlierPts[:, 1], color='blue', label='Filtered observations (Inliers)')
            plt.scatter(outlierPts[:, 0], outlierPts[:, 1], color='red', label='Filtered observations (Outliers)')

            # Display the final fit line. This line is robust to the outliers that ransac identified and ignored.
            plt.plot(x, y, color='green', linewidth=3, label='RANSAC regression line')

            # Odometry
            plt.scatter(odoms[:, 0], odoms[:, 1], color='k', label='Odometry')
            plt.scatter(rob_obs_pose[0], rob_obs_pose[1], color='m', marker='o', label='Robot observation pose')

            plt.legend()
            plt.xlabel('x [mm]')
            plt.ylabel('y [mm]')
            plt.axis('equal')

            # for annotating the line parameters
            plt.plot([0, x0], [0, y0], color='green')
            plt.plot([x[0], x0], [y[0], y0], color='green')
            plt.plot([x[1], x0], [y[1], y0], color='green')

            plt.show()

    return lineParams_robot, outlierPts, inlierPts, isLine

def fitransac2(points, odoms, rob_obs_pose, L_thresh_n, D_ransac, fig):
    sampleSize = 3  # number of points to sample per trial
    # maxDistance = 15  # max allowable distance for inliers
    # maxDistance = D_ransac # max allowable distance for inliers

    lineParams_robot = None
    outlierPts = np.array([0])
    inlierPts = np.array([0])
    # print("fitransac_2 running")
    isLine, inlierIdx, modelInliers = myRANSAC(points, sampleSize, D_ransac, L_thresh_n)

    isLine, inlierIdx, modelInliers = modify_lineParams(isLine, inlierIdx, modelInliers, 70, points, sampleSize, D_ransac, L_thresh_n)

    if isLine:
        # Inliers
        inlierPts = points[inlierIdx, :]

        # Outliers
        outlierPts = points[~inlierIdx, :]

        # Refit a line to the inliers using polyfit.
        # modelInliers = np.polyfit(points[inlierIdx, 0], points[inlierIdx, 1], 1)

        # Line parameters::: y = mx + c
        m = modelInliers[0]
        c = modelInliers[1]

        x0 = -c * m / (m ** 2 + 1)
        y0 = -x0 / m

        # Line start/end points
        x = [np.min(inlierPts[:, 0]), np.max(inlierPts[:, 0])]
        y = [np.min(inlierPts[:, 1]), np.max(inlierPts[:, 1])]
        

        # Line params from the origin
        r = np.sqrt(x0 ** 2 + y0 ** 2)
        th = np.arctan2(y0, x0)
        len_ = np.sqrt((x[0] - x[1]) ** 2 + (y[0] - y[1]) ** 2)

        # Line params as seen from the robot
        odompt = rob_obs_pose

        isInter = chkIntersec2(rob_obs_pose, m, c)  # check the intersecting condition

        if isInter:
            r0 = -r + odompt[0] * np.cos(th) + odompt[1] * np.sin(th)
            th0 = th - odompt[2] + np.pi
        else:
            r0 = r - odompt[0] * np.cos(th) - odompt[1] * np.sin(th)
            th0 = th - odompt[2]

        lineParams_robot = [r0.item(), th0.item(), len_, m, c, x[0], x[1], y[0], y[1]] # line parameters as seen from the robot [r0, th0, len, m, c, x0, x1, y0, y1]

        # Plotting
        if fig:
            import matplotlib.pyplot as plt

            plt.figure()

            plt.scatter(inlierPts[:, 0], inlierPts[:, 1], color='blue')
            plt.scatter(outlierPts[:, 0], outlierPts[:, 1], color='red')

            plt.plot(x, y, 'g-', linewidth=3)

            plt.scatter(odoms[:, 0], odoms[:, 1], color='k')
            plt.scatter(rob_obs_pose[0], rob_obs_pose[1], color='m', marker='o')

            plt.legend(['RANSAC regression line', 'Odometry', 'Robot observation pose'])

            plt.plot([0, x0], [0, y0], 'green')
            plt.plot([x[0], x0], [y[0], y0], 'green')
            plt.plot([x[1], x0], [y[1], y0], 'green')

            plt.axis('equal')
            plt.xlabel('x [mm]')
            plt.ylabel('y [mm]')
            plt.show()

    return lineParams_robot, outlierPts, inlierPts, isLine

# Extract lines from the observations
def lineExtract(pts_i, odom_i, rob_obs_pose, thresh, N_LMs, fig):
    N_inl_thresh, L_len_thresh, r_thresh, th_thresh, D_ransac = thresh
    outlierPts = pts_i
    L_LMs = np.zeros((6, 9))  # Initialize array to store 6 line parameters

    if pts_i.shape[0] > N_inl_thresh:
        lineParams_robot, tmp_outlierPts, inlierPts, isLine = fitransac(pts_i, odom_i, rob_obs_pose, L_len_thresh//50, D_ransac, fig)
        N_inliers = inlierPts.shape[0]

        if N_inliers > N_inl_thresh and lineParams_robot[2] > L_len_thresh and isLine:
            N_LMs[0] += 1
            L_LMs[0] = lineParams_robot
            outlierPts = tmp_outlierPts

            # If a line is found, check for more lines in the outliers
            while tmp_outlierPts.shape[0] > N_inl_thresh:
                lineParams_robot, tmp_outlierPts, inlierPts, isLine = fitransac2(tmp_outlierPts, odom_i, rob_obs_pose, L_len_thresh//50, D_ransac, fig)
                N_inliers = inlierPts.shape[0]
                if N_inliers > N_inl_thresh and lineParams_robot[2] > L_len_thresh and isLine:
                    # print("lineParams_robot[0]: ", lineParams_robot[0])
                    # print("N_LMs[0]: ", N_LMs[0])
                    # print("L_LMs[:N_LMs[0], 0]: ", L_LMs[:N_LMs[0], 0])
                    # print('lineParams_robot[1]: ', lineParams_robot[1])
                    # print('L_LMs[:N_LMs[0], 1]: ', L_LMs[:N_LMs[0], 1])
                    check = np.any(np.abs(lineParams_robot[0] - L_LMs[:N_LMs[0], 0]) < r_thresh) or \
                            np.any(np.abs(lineParams_robot[1] - L_LMs[:N_LMs[0], 1]) < th_thresh)

                    if not check or N_LMs[0] == 0:
                        L_LMs[N_LMs[0]] = lineParams_robot
                        N_LMs[0] += 1 
                        outlierPts = tmp_outlierPts         

    return N_LMs, L_LMs, outlierPts

#_______________________________________________________________________________________________________________________
#
# Point Extraction
#_______________________________________________________________________________________________________________________

# Filter points that are close to each other
def filtnearPts(P_LMs, N_LMs, pts):
    thresh = 300
    D = np.linalg.norm(pts[:, np.newaxis, :] - pts[np.newaxis, :, :], axis=-1) # distance of the point to all other points
    n = N_LMs[1]
    p_LMs = np.empty((0, 2))
    n_LMs = 0

    for i in range(n): # for each point
        d = D[i] # distance of the point to all other points   
        if np.all(d[d != 0] > thresh): # check if the distance of the point to all other points is greater than the threshold
            p_LMs = np.vstack((p_LMs, P_LMs[i])) # add the point to the list of points
            n_LMs += 1 # increment the number of points

    N_LMs[1] = n_LMs
    return p_LMs, N_LMs

# Extract points from the observations
def pointExtract(points, L_LMs, N_LMs, d_thresh, params, odom_i, rob_obs_pose, fig):
    P_LMs = np.zeros((2))
    xymeans = np.zeros((2))
    if points.shape[0] >= params[0]: # more than 5 points, preferably min_samples=params[0]
        cluster1 = DBSCAN(eps=params[1], min_samples=params[0], algorithm='auto') # clustering algorithm
        clus_idx = cluster1.fit_predict(points) # cluster the points

        N = np.max(clus_idx) + 1 # number of clusters

        xymeans = np.zeros((N, 2))
        P_LMs = np.zeros((N, 2))
        # lengths = np.zeros((N, N_LMs[0]))

        for i in range(N): # for each cluster
            ind = np.where(clus_idx == i)[0] # get the indices of the points in the cluster
            pt = np.mean(points[ind], axis=0) # get the mean of the points in the cluster

            check = True # flag to check if the point is close to a line
            for j in range(N_LMs[0]): # for each line
                D = pt2line(L_LMs[j, 3:5], pt) # distance of the point to the line || 3:5 to get [m, c] from [r0, th0, length, m, c, x0, x1, y0, y1]
                # lengths[N_LMs[1], j] = D
                if D < d_thresh: # check if the distance is less than the threshold
                    check = False
                    break

            if check: # if the point is not close to any line
                xymeans[N_LMs[1]] = pt
                N_LMs[1] += 1

                odompt = rob_obs_pose.flatten()
                X = np.vstack((pt, odompt[:2]))
                r = np.linalg.norm(X[0] - X[1])
                P_LMs[N_LMs[1] - 1, 0] = r

                del_pt = pt - odompt[:2]
                th = np.arctan2(del_pt[1], del_pt[0]) - odompt[2]
                P_LMs[N_LMs[1] - 1, 1] = th

        if N_LMs[1] > 1:
            P_LMs, N_LMs = filtnearPts(P_LMs, N_LMs, xymeans)

        if fig:
            plt.figure()
            plt.scatter(odom_i[:, 0], odom_i[:, 1], marker='.', color='black', label='Odometry')
            plt.scatter(xymeans[:N_LMs[1], 0], xymeans[:N_LMs[1], 1], color='blue', label='Cluster center(s)')
            plt.legend()
            plt.xlabel('x [mm]')
            plt.ylabel('y [mm]')
            plt.axis('equal')
            plt.show()

    return N_LMs, P_LMs, xymeans

#_______________________________________________________________________________________________________________________
#
# FINAL Line and Point Extraction
#_______________________________________________________________________________________________________________________

def createObs(N_LMs_lhs_1, L_LMs_lhs_1, P_LMs_lhs_1, N_LMs_lhs_12, L_LMs_lhs_12,
              N_LMs_rhs_1, L_LMs_rhs_1, P_LMs_rhs_1, N_LMs_rhs_12, L_LMs_rhs_12,
              odom, mu, mu_bar, N_line, N_pts_allocated, 
              visLine_x, visLine_y, plotfig, ax_L, ax_P):
    
    # odompt = mu_bar[:3]
    odompt = odom.reshape(-1, 1)

    pt2line_thresh = 250

    obs_Pts = np.zeros((10, 4)) # 10 points with 4 parameters each >> [x, y, idx, side = 1 for LHS and 2 for RHS]
    obs_Lins = np.zeros((6, 9)) # 6 lines with 9 parameters each

    count_L = 0
    count_P = 0

    '''
    The line landmarks are directly added to the observation list.
    '''
    # LHS
    for i in range(N_LMs_lhs_1[0]):
        obs_Lins[count_L, :] = L_LMs_lhs_1[i, :]
        count_L += 1
    # RHS
    for i in range(N_LMs_rhs_1[0]):
        obs_Lins[count_L, :] = L_LMs_rhs_1[i, :]
        count_L += 1

    '''
    The point landmarks are added to the observation list if they are not close to any line landmarks.
    '''
    # LHS
    mem_1 = False
    mem_2 = False
    for i in range(N_LMs_lhs_1[1]): # for each point landmark observation in REGION 1
        x0 = odompt[0] + P_LMs_lhs_1[i, 0] * np.cos(P_LMs_lhs_1[i, 1] + odompt[2]) # with respect to the robot ODOM
        y0 = odompt[1] + P_LMs_lhs_1[i, 0] * np.sin(P_LMs_lhs_1[i, 1] + odompt[2])
        x1, y1 = P_LMs_lhs_1[i, :2]
        prev_dist = 1e10
        for j in range(N_LMs_lhs_12[0]): # for each line landmark observation in REGION 12
            m, c = L_LMs_lhs_12[j, 3:5]
            pt2lin_dist = pt2line([m, c], [x0, y0])
            if pt2lin_dist < pt2line_thresh:
                mem_1 = True
                if pt2lin_dist < prev_dist:
                    near_line = L_LMs_lhs_12[j, :]
                    prev_dist = pt2lin_dist

        for k in range(N_line): # for all initialized lines
            # print('k: ', k)
            m, c = rth2mc(mu_bar, N_pts_allocated, k)
            # print('dist: ', pt2line([m, c], [_x0, _y0]))
            # print('[m, c]: ', [m, c])
            # print('[_x0, _y0]: ', [_x0, _y0])
            # check if the point is close to the line
            if pt2line([m, c], [x0, y0]) < pt2line_thresh: # with respect to the robot mu_bar (Estimate) because the initialized estimated lines
                mid = [np.mean(visLine_x[k]), np.mean(visLine_y[k])]
                # print('mid: ', mid)
                mid2pt = np.linalg.norm([mid[0] - x0, mid[1] - y0])
                # print('mid2pt: ', mid2pt)
                ln_len = np.linalg.norm([visLine_x[k, 0] - visLine_x[k, 1], visLine_y[k, 0] - visLine_y[k, 1]])
                # print('ln_len: ', ln_len)
                if mid2pt < ln_len / 2 + 300:
                    mem_2 = True

        if mem_1:
            # print("LHS line observed")
            obs_Lins[count_L, :] = near_line
            count_L += 1
        elif not mem_1 and not mem_2:
            # print("LHS point observed")
            obs_Pts[count_P, :2] = [x1, y1]
            obs_Pts[count_P, 2] = count_P
            obs_Pts[count_P, 3] = 1 # side = 1 for LHS
            count_P += 1
        mem_1 = False
        mem_2 = False

    # RHS
    mem_3 = False
    mem_4 = False
    for i in range(N_LMs_rhs_1[1]): # for each point landmark observation in REGION 1
        x0 = odompt[0] + P_LMs_rhs_1[i, 0] * np.cos(P_LMs_rhs_1[i, 1] + odompt[2]) # with respect to the robot ODOM
        y0 = odompt[1] + P_LMs_rhs_1[i, 0] * np.sin(P_LMs_rhs_1[i, 1] + odompt[2])
        x1, y1 = P_LMs_rhs_1[i, :2]
        prev_dist = 1e10
        for j in range(N_LMs_rhs_12[0]): # for each line landmark observation in REGION 12
            m, c = L_LMs_rhs_12[j, 3:5]
            pt2lin_dist = pt2line([m, c], [x0, y0])
            if pt2lin_dist < pt2line_thresh:
                mem_3 = True
                if pt2lin_dist < prev_dist:
                    near_line = L_LMs_rhs_12[j, :]

        for k in range(N_line): # for all initialized lines
            m, c = rth2mc(mu_bar, N_pts_allocated, k)
            # check if the point is close to the line
            if pt2line([m, c], [x0, y0]) < pt2line_thresh:
                mid = [np.mean(visLine_x[k]), np.mean(visLine_y[k])]
                mid2pt = np.linalg.norm([mid[0] - x0, mid[1] - y0])
                ln_len = np.linalg.norm([visLine_x[k, 0] - visLine_x[k, 1], visLine_y[k, 0] - visLine_y[k, 1]])
                if mid2pt < ln_len / 2 + 300:
                    mem_4 = True

        if mem_3:
            # print("RHS line observed")
            obs_Lins[count_L, :] = near_line
            count_L += 1
        elif not mem_3 and not mem_4:
            # print("RHS point observed")
            obs_Pts[count_P, :2] = [x1, y1]
            obs_Pts[count_P, 2] = count_P
            obs_Pts[count_P, 3] = 2
            count_P += 1
        mem_3 = False
        mem_4 = False

    if plotfig:
        visPLs([count_L, count_P], obs_Lins, obs_Pts, odompt, ax_L, ax_P)
    return obs_Lins, obs_Pts

# Convert r and th line parameters to m and c
def rth2mc(mu, nPtLMs, n):
    lin_i = nPtLMs * 2 + 3 + 2 * n # index of the line parameters
    
    line_r = mu[lin_i]
    line_th = mu[lin_i + 1]

    m = np.tan(np.pi / 2 + line_th)
    c = line_r / np.sin(line_th)

    return m, c

# Convert r and th line parameters to m and c
def rth2mc2(mu, r, th):
    psi = th + mu[2]

    x = r * math.cos(psi) + mu[0]
    y = r * math.sin(psi) + mu[1]

    line_r = math.sqrt(x ** 2 + y ** 2)
    line_th = math.atan2(y, x)

    m = math.tan(math.pi / 2 + line_th)
    c = line_r / math.sin(line_th)

    return m, c

# Convert r0 and alpha line parameters to m and c
def roalp2mc(mu, ro, alp):
    psi = alp + mu[2]

    x1 = ro * np.cos(psi) + mu[0]
    y1 = ro * np.sin(psi) + mu[1]

    m = np.tan(np.pi / 2 + psi)
    c = y1 - m * x1

    return m, c

# Visualize the line and point landmarks
def visPLs(N_LMs, L_LMs, P_LMs, odompt, ax_L, ax_P):
    # Lines
    for i in range(N_LMs[0]):
        l_x = L_LMs[i, 5:7]
        l_y = L_LMs[i, 7:9]
        # if L_LMs[i, 2] < 0:
        #     l_x = np.flip(l_x)
        ax_L[i].set_xdata(l_x)
        ax_L[i].set_ydata(l_y)

    # Points
    p_x = odompt[0] + P_LMs[:N_LMs[1], 0] * np.cos(P_LMs[:N_LMs[1], 1] + odompt[2])
    p_y = odompt[1] + P_LMs[:N_LMs[1], 0] * np.sin(P_LMs[:N_LMs[1], 1] + odompt[2])

    ax_P.set_offsets(np.column_stack((p_x, p_y)))

# Visualize the line and point landmarks
def visPLs2(N_LMs, L_LMs, P_LMs, ax_L, ax_P, fig):
    # Lines
    # L_LMs = [r, psi, m, c, x0, x1, y0, y1]
    for i in range(N_LMs[0]):
        l_x = L_LMs[i, 5:7]
        l_y = L_LMs[i, 7:9]

        # if L_LMs[i, 3] < 0:
        #     l_x = np.flip(l_x)
        ax_L.set_xdata(l_x)
        ax_L.set_ydata(l_y)
        # m = L_LMs[i, 2]
        # c = L_LMs[i, 3]
        # ax.set_ydata([m * l_x[0] + c, m * l_x[1] + c])
        fig.canvas.draw()

    if N_LMs[0] == 0:
        ax_L.set_xdata([])
        ax_L.set_ydata([])
        fig.canvas.draw()
    
    # Points
    for i in range(N_LMs[1]):
        ax_P.set_offsets(P_LMs[i, :2])
        fig.canvas.draw()
    
    if N_LMs[1] == 0:
        ax_P.set_offsets([ None, None])
        fig.canvas.draw()

#_______________________________________________________________________________________________________________________
#
# EKF SLAM
#_______________________________________________________________________________________________________________________

# Predict the state and covariance
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

# POINTS:: Correct the state and covariance 
def EKF_unknown_pts_obs_correction(k, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q, z):
    dx = mu_bar[3 + 2 * k] - mu_bar[0]
    dy = mu_bar[4 + 2 * k] - mu_bar[1]
    del_ = [dx[0], dy[0]]

    q = np.dot(del_, del_)
    sq = np.sqrt(q)

    z0 = np.array([sq, np.arctan2(del_[1], del_[0]) - mu_bar[2][0]]).reshape(-1, 1)

    if z[1] > np.pi and z0[1] < 0:
        z0[1] = 2 * np.pi + z0[1]

    elif z0[1] > np.pi and z[1] < 0:
        z0[1] = z0[1] - 2 * np.pi

    elif z0[1] < -np.pi and z[1] > 0:
        z0[1] = z0[1] + 2 * np.pi
    
    elif z[1] < -np.pi and z0[1] > 0:
        z[1] = z[1] + 2 * np.pi

    del_z = z - z0

    F_xk = np.block([
        [np.eye(3), np.zeros((3, 2 * k)), np.zeros((3, 2)), np.zeros((3, 2 * exp_pt_landm - 2 * k - 2)), np.zeros((3, 2 * exp_line_landm))],
        [np.zeros((2, 3)), np.zeros((2, 2 * k)), np.eye(2), np.zeros((2, 2 * exp_pt_landm - 2 * k - 2)), np.zeros((2, 2 * exp_line_landm))]])

    H = (1 / q) * np.array([[-del_[0] * sq, -del_[1] * sq, 0, del_[0] * sq, del_[1] * sq],
                            [del_[1], -del_[0], -1, -del_[1], del_[0]]]) @ F_xk

    psi = H @ sig_bar @ H.T + Q
    pie = del_z.T @ np.linalg.inv(psi) @ del_z

    return pie, psi, H, z0

# LINES:: Correct the state and covariance || depends on the line model:: intersects or not, read paper (refer Fig. #)
def EKF_unknown_line_obs_correction_1(k, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q, z):
    r_i = mu_bar[3 + 2 * exp_pt_landm + 2 * k]
    psi_i = mu_bar[4 + 2 * exp_pt_landm + 2 * k]

    z0 = np.array([r_i - mu_bar[0] * np.cos(psi_i) - mu_bar[1] * np.sin(psi_i), psi_i - mu_bar[2]])
    # print('z0: ', z0)
    F_xk = np.block([
        [np.eye(3), np.zeros((3, 2 * exp_pt_landm)), np.zeros((3, 2 * k)), np.zeros((3, 2)), np.zeros((3, 2 * exp_line_landm - 2 * k - 2))],
        [np.zeros((2, 3)), np.zeros((2, 2 * exp_pt_landm)), np.zeros((2, 2 * k)), np.eye(2), np.zeros((2, 2 * exp_line_landm - 2 * k - 2))]])

    H = np.array([[-math.cos(psi_i), -math.sin(psi_i), 0, 1, mu_bar[0][0] * math.sin(psi_i) - mu_bar[1][0] * math.cos(psi_i)],
                  [0, 0, -1, 0, 1]]) @ F_xk

    psi = H @ sig_bar @ H.T + Q

    if z[1] > np.pi and z0[1] < 0:
        z0[1] = 2 * np.pi + z0[1]

    elif z0[1] > np.pi and z[1] < 0:
        z0[1] = z0[1] - 2 * np.pi

    elif z0[1] < -np.pi and z[1] > 0:
        z0[1] = z0[1] + 2 * np.pi
    
    elif z[1] < -np.pi and z0[1] > 0:
        z[1] = z[1] + 2 * np.pi

    del_Z = z - z0
    # print('del_Z: ', del_Z)

    pie = np.dot(del_Z.T, np.linalg.inv(psi)) @ del_Z
    m_r = del_Z[0]**2 / psi[0, 0]
    m_al = del_Z[1]**2 / psi[1, 1]
    return pie, psi, H, z0, m_r, m_al
    # return pie, psi, H, z0,

# LINES:: Correct the state and covariance || depends on the line model:: intersects or not, read paper (Fig. #)
def EKF_unknown_line_obs_correction_2(k, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q, z):
    r_i = mu_bar[3 + 2 * exp_pt_landm + 2 * k]
    psi_i = mu_bar[4 + 2 * exp_pt_landm + 2 * k]

    z0 = np.array([-r_i + mu_bar[0] * np.cos(psi_i) + mu_bar[1] * np.sin(psi_i), psi_i - mu_bar[2] + np.pi]).reshape(-1, 1)
    # print('z0: ', z0)

    F_xk = np.block([[np.eye(3), np.zeros((3, 2 * exp_pt_landm)), np.zeros((3, 2 * k)), np.zeros((3, 2)), np.zeros((3, 2 * exp_line_landm - 2 * k - 2))],
                      [np.zeros((2, 3)), np.zeros((2, 2 * exp_pt_landm)), np.zeros((2, 2 * k)), np.eye(2), np.zeros((2, 2 * exp_line_landm - 2 * k - 2))]])

    H = np.array([[math.cos(psi_i), math.sin(psi_i), 0, -1, mu_bar[1][0] * math.cos(psi_i) - mu_bar[0][0] * math.sin(psi_i)],
                  [0, 0, -1, 0, 1]]) @ F_xk

    psi = H @ sig_bar @ H.T + Q

    if z[1] > np.pi and z0[1] < 0:
        z0[1] += 2 * np.pi

    elif z0[1] > np.pi and z[1] < 0:
        z0[1] -= 2 * np.pi

    elif z0[1] < -np.pi and z[1] > 0:
        z0[1] = z0[1] + 2 * np.pi

    elif z[1] < -np.pi and z0[1] > 0:
        z[1] = z[1] + 2 * np.pi

    del_Z = z - z0
    # print('del_Z: ', del_Z)

    pie = (del_Z.T @ np.linalg.inv(psi) @ del_Z).item()
    m_r = del_Z[0]**2 / psi[0, 0]
    m_al = del_Z[1]**2 / psi[1, 1]
    return pie, psi, H, z0, m_r, m_al
    # return pie, psi, H, z0

# EKF correction for both point and line landmarks
def EKF_unknown_correction_LP(mu_bar, sig_bar, obs_pts, obs_lin, all_lhs_pts, all_rhs_pts, Q_pts, Q_lin, iter, hist_i, countarr, exp_pt_landm, exp_line_landm, N_pt, N_line, visLine_x, visLine_y, alp_pt, alp_line, alp_C):
    obs_pts = np.squeeze(obs_pts)
    obs_lin = np.squeeze(obs_lin)
    len_obs_pts, _ = obs_pts.shape
    len_obs_lin, _ = obs_lin.shape
    # ind_j_pt = -1
    # ind_j_ln = -1
    # Point landmarks
    for j in range(len_obs_pts):
        if obs_pts[j, 0] != 0:
            r = obs_pts[j, 0]
            phi = obs_pts[j, 1]
            side = obs_pts[j, 3] # side = 1 for LHS and 2 for RHS

            z = np.array([r, phi]).reshape(-1, 1)

            # rel_meas = np.array([r * np.cos(phi + mu_bar[2]), r * np.sin(phi + mu_bar[2])]).reshape(-1, 1)

            cent = np.array([mu_bar[0] + r * np.cos(phi + mu_bar[2]), mu_bar[1] + r * np.sin(phi + mu_bar[2])])

            if side == 1: # LHS
                iso = chkIsolated(np.array(all_lhs_pts), cent, 200, 300)
            elif side == 2: # RHS
                iso = chkIsolated(np.array(all_rhs_pts), cent, 200, 300)

            if iso:
                mu_bar[1 + 2 * (N_pt + 1):3 + 2 * (N_pt + 1)] = cent

                pie = np.zeros(N_pt + 1)
                for k in range(N_pt):
                    pie[k], _, _, _ = EKF_unknown_pts_obs_correction(k, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q_pts, z)
                pie[N_pt] = alp_pt
                
                ind_j_pt = np.argmin(pie)

                _, psi_j, H_j, z_j = EKF_unknown_pts_obs_correction(ind_j_pt, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q_pts, z)

                N_pt = max(N_pt, ind_j_pt + 1)
                print('\nPoints pie: ', pie)
                K = sig_bar @ H_j.T @ np.linalg.inv(psi_j)

                mu_bar = mu_bar + K @ (z - z_j)
                sig_bar = (np.eye(sig_bar.shape[0]) - K @ H_j) @ sig_bar

                hist_i[ind_j_pt] = iter # update the history of the point landmark's observation index

                # mu_bar, sig_bar, N_pt, countarr, hist_i = modifyPtLMs(iter, hist_i, countarr, [exp_pt_landm, exp_line_landm], 1, ind_j_pt, mu_bar, sig_bar, N_pt)
        else:
            break

    # Line landmarks
    for j in range(len_obs_lin):
        if obs_lin[j, 2] != 0:
            ro = obs_lin[j, 0]
            alpha = obs_lin[j, 1]
            z = np.array([ro, alpha]).reshape(-1, 1)
            print('\n --------------------------------------------------------------------------')
            isInter = chkIntersec(mu_bar, ro, alpha)

            if isInter:
                gam = alpha + mu_bar[2] - np.pi
                r = mu_bar[0] * np.cos(gam) + mu_bar[1] * np.sin(gam) - ro
                psi = gam
            else:
                gam = alpha + mu_bar[2]
                r = ro + mu_bar[0] * np.cos(gam) + mu_bar[1] * np.sin(gam)
                psi = gam
            
            mu_bar[1 + 2 * exp_pt_landm + 2 * (N_line + 1): 3 + 2 * exp_pt_landm + 2 * (N_line + 1)] = np.array([r, psi]).reshape(-1, 1)

            pie = np.zeros(N_line + 1)
            m_r = np.zeros(N_line + 1)
            m_al = np.zeros(N_line + 1)
            for k in range(N_line):
                r = mu_bar[3 + 2 * exp_pt_landm + 2 * k]
                psi = mu_bar[4 + 2 * exp_pt_landm + 2 * k]
                isInter = chkIntersec3(mu_bar, r, psi) # Check if the line intersects with the robot's path
                if isInter:
                    pie[k], _, _, _, m_r[k], m_al[k] = EKF_unknown_line_obs_correction_2(k, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q_lin, z)
                else:
                    pie[k], _, _, _, m_r[k], m_al[k] = EKF_unknown_line_obs_correction_1(k, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q_lin, z)
            
            pie[N_line] = alp_line

            # Find nearby lines
            if N_line > 0:
                D, _, _ = findNearbyLines(mu_bar[0:3], z, N_line, visLine_x, visLine_y)
                print('\nDistances: ', D)

            print('\nLines pie: ', pie)
            # print('\nLines m_r: ', m_r)
            # print('\nLines m_al: ', m_al)
            ind_j_ln = np.argmin(pie)
            print('old_ind_j_ln: ', ind_j_ln)
            hist_i[ind_j_ln + exp_pt_landm] = iter # update the history of the line landmark's observation index

            ind_bel_thresh = get_indexes_below_threshold(pie, alp_line) # There can be more than one LANDMARKS below the M dist threshold

            # loop closure with previously observed lines
            if ind_j_ln == N_line and N_line > 10: # if a new landmark is observed
                pie[N_line -10:] = 100 # set the last 10 landmarks to a high value
                # pdb.set_trace()
                min_pie = np.min(pie)
                ind_min_pie = np.argmin(pie)
                if min_pie < alp_C and D[ind_min_pie] < 5000: # if the new landmark is close to any of the last 10 landmarks with 3 mahalanobis distance
                    ind_j_ln = np.argmin(pie)   
            
            elif ind_j_ln < N_line: # if not a new landmark::
                len_ind_bel_thresh = len(ind_bel_thresh)
                if len_ind_bel_thresh == 1: # if there is only one landmark below the M dist threshold
                    if D[ind_j_ln] > 2700: # minimum distance between two lines
                        ind_j_ln = N_line # set the new landmark as a new landmark

                else: # if there are more than one landmarks below the M dist threshold
                    pie_vals = [pie[i] for i in ind_bel_thresh] # get the distances of the landmarks below the M dist threshold
                    ind_bel_thresh = sort_two_lists(pie_vals, ind_bel_thresh) # sort the pie_vals and the indexes
                    for ind in ind_bel_thresh:  # go through all the landmarks below the M dist threshold
                        if D[ind] < 2700: # minimum distance between two lines
                            ind_j_ln = ind
                            break
                        else:
                            ind_j_ln = N_line

            print('new_ind_j_ln: ', ind_j_ln)


            r = mu_bar[3 + 2 * exp_pt_landm + 2 * ind_j_ln]
            psi = mu_bar[4 + 2 * exp_pt_landm + 2 * ind_j_ln]
            isInter = chkIntersec3(mu_bar, r, psi)

            # # Adding the constraints for the line landmarks
            # ub = 13000
            # lb = 12000
            # if iter == 3939:
            #     ub = 15000
            #     lb = 12000
            # if lb < r < ub and mu_bar[3 + 2 * exp_pt_landm + 1] -0.1 < psi < mu_bar[3 + 2 * exp_pt_landm + 1] + 0.1:
            #     # pdb.set_trace()
            #     mu_bar[2] = mu_bar[2] + mu_bar[2 + 2*exp_pt_landm + 2*1] - mu_bar[2 + 2*exp_pt_landm + 2*9]  # adjust the orientation
            #     isInter = chkIntersec(mu_bar, ro, alpha)

            #     if isInter:
            #         gam = alpha + mu_bar[2] - np.pi
            #         r = mu_bar[0] * np.cos(gam) + mu_bar[1] * np.sin(gam) - ro
            #         psi = gam
            #     else:
            #         gam = alpha + mu_bar[2]
            #         r = ro + mu_bar[0] * np.cos(gam) + mu_bar[1] * np.sin(gam)
            #         psi = gam
                
            #     mu_bar[1 + 2 * exp_pt_landm + 2 * (N_line + 1): 3 + 2 * exp_pt_landm + 2 * (N_line + 1)] = np.array([r, psi]).reshape(-1, 1)
            # -----------------------------------------------------------------------------------------------

            if isInter:
                _, psi_j, H_j, z_j, _, _ = EKF_unknown_line_obs_correction_2(ind_j_ln, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q_lin, z)
            else:
                _, psi_j, H_j, z_j, _, _ = EKF_unknown_line_obs_correction_1(ind_j_ln, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q_lin, z)
            
            # comment the following two lines if you are using this function for simulation
            # also remove else >> break parts in both line and point sections if you are using this function for simulation
            visLine_x = updateLimsX(ind_j_ln, visLine_x, [obs_lin[j, 5], obs_lin[j, 6]])
            visLine_y = updateLimsY(ind_j_ln, visLine_y, [obs_lin[j, 7], obs_lin[j, 8]])

            N_line = max(N_line, ind_j_ln + 1)

            K = sig_bar @ H_j.T @ np.linalg.inv(psi_j)
            # print('del_z: ', z - z_j)
            delta_mu = K @ (z - z_j)
            mu_bar = mu_bar + delta_mu
            sig_bar = (np.eye(sig_bar.shape[0]) - K @ H_j) @ sig_bar

        else:
            break

    return mu_bar, sig_bar, N_pt, N_line, hist_i, countarr, visLine_x, visLine_y

# EKF correction for both point and line landmarks
### ONLY FOR THE SIMULATION !!!
def EKF_unknown_correction_LP2(mu_bar, sig_bar, obs_pts, obs_lin, Q_pts, Q_lin, iter, hist_i, countarr, exp_pt_landm, exp_line_landm, N_pt, N_line, visLine_x, visLine_y, alp_pt, alp_line):
    obs_pts = np.squeeze(obs_pts)
    obs_lin = np.squeeze(obs_lin)
    len_obs_pts, _ = obs_pts.shape
    len_obs_lin, _ = obs_lin.shape
    print('obs_pts: ', obs_pts)
    print('obs_lin: ', obs_lin)
    # ind_j_pt = -1
    # ind_j_ln = -1
    # Point landmarks
    for j in range(len_obs_pts):
        if obs_pts[j, 0] != 0:
            r = obs_pts[j, 0]
            phi = obs_pts[j, 1]
            z = np.array([r, phi]).reshape(-1, 1)

            rel_meas = np.array([r * np.cos(phi + mu_bar[2]), r * np.sin(phi + mu_bar[2])]).reshape(-1, 1)
            mu_bar[1 + 2 * (N_pt + 1):3 + 2 * (N_pt + 1)] = mu_bar[:2] + rel_meas

            pie = np.zeros(N_pt + 1)
            for k in range(N_pt):
                pie[k], _, _, _ = EKF_unknown_pts_obs_correction(k, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q_pts, z)
            pie[N_pt] = alp_pt
            
            ind_j_pt = np.argmin(pie)
            _, psi_j, H_j, z_j = EKF_unknown_pts_obs_correction(ind_j_pt, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q_pts, z)

            N_pt = max(N_pt, ind_j_pt + 1)
            print('\nPoints pie: ', pie)
            K = sig_bar @ H_j.T @ np.linalg.inv(psi_j)

            mu_bar = mu_bar + K @ (z - z_j)
            sig_bar = (np.eye(sig_bar.shape[0]) - K @ H_j) @ sig_bar

            # mu_bar, sig_bar, N_pt, countarr, hist_i = modifyPtLMs(iter, hist_i, countarr, [exp_pt_landm, exp_line_landm], 1, ind_j_pt, mu_bar, sig_bar, N_pt)

    # Line landmarks
    for j in range(len_obs_lin):
        if obs_lin[j, 2] != 0:
            ro = obs_lin[j, 0]
            alpha = obs_lin[j, 1]
            z = np.array([ro, alpha]).reshape(-1, 1)

            isInter = chkIntersec(mu_bar, ro, alpha)

            if isInter:
                gam = alpha + mu_bar[2] - np.pi
                r = mu_bar[0] * np.cos(gam) + mu_bar[1] * np.sin(gam) - ro
                psi = gam
            else:
                gam = alpha + mu_bar[2]
                r = ro + mu_bar[0] * np.cos(gam) + mu_bar[1] * np.sin(gam)
                psi = gam
            
            mu_bar[1 + 2 * exp_pt_landm + 2 * (N_line + 1): 3 + 2 * exp_pt_landm + 2 * (N_line + 1)] = np.array([r, psi]).reshape(-1, 1)

            pie = np.zeros(N_line + 1)
            for k in range(N_line):
                r = mu_bar[3 + 2 * exp_pt_landm + 2 * k]
                psi = mu_bar[4 + 2 * exp_pt_landm + 2 * k]
                isInter = chkIntersec3(mu_bar, r, psi) # Check if the line intersects with the robot's path
                pie[k], _, _, _, _, _ = EKF_unknown_line_obs_correction_1(k, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q_lin, z)
            
            print('\nLines pie: ', pie)
            pie[N_line] = alp_line
            ind_j_ln = np.argmin(pie)

            r = mu_bar[3 + 2 * exp_pt_landm + 2 * ind_j_ln]
            psi = mu_bar[4 + 2 * exp_pt_landm + 2 * ind_j_ln]
            _, psi_j, H_j, z_j, _, _ = EKF_unknown_line_obs_correction_1(ind_j_ln, mu_bar, sig_bar, exp_pt_landm, exp_line_landm, Q_lin, z)

            N_line = max(N_line, ind_j_ln + 1)

            K = sig_bar @ H_j.T @ np.linalg.inv(psi_j)

            delta_mu = K @ (z - z_j)

            if abs(delta_mu[0]) < 200 and abs(delta_mu[1]) < 200:
                mu_bar = mu_bar + delta_mu
                sig_bar = (np.eye(sig_bar.shape[0]) - K @ H_j) @ sig_bar
                # mu_bar, sig_bar, N_line, countarr, hist_i, visLine_x, visLine_y = modifyLinLMs(iter, hist_i, countarr, [exp_pt_landm, exp_line_landm], 20, ind_j_ln, mu_bar, sig_bar, N_line, N_line + 1, z, visLine_x, visLine_y)

    return mu_bar, sig_bar, N_pt, N_line, hist_i, countarr, visLine_x, visLine_y

# Modify the mu and sigma matrices
def modify_mu_sig(mu, Sig, ids, exp_LM):
    n_pos = 3
    n_pts = exp_LM[0]
    n_ptSq = n_pts * 2
    n_lin = exp_LM[1]
    n_lnSq = n_lin * 2

    pt_id = ids[0]
    pt_idSq = pt_id * 2
    ln_id = ids[1]
    ln_idSq = ln_id * 2

    if pt_id > 0:
        Sig[pt_idSq - 1 + n_pos: n_pos + n_ptSq - 1, :] = Sig[pt_idSq + 1 + n_pos: n_pos + n_ptSq, :]
        Sig[:, pt_idSq - 1 + n_pos: n_pos + n_ptSq - 1] = Sig[:, pt_idSq + 1 + n_pos: n_pos + n_ptSq]
        Sig[n_pos + n_ptSq - 1: n_pos + n_ptSq, :] = 0
        Sig[:, n_pos + n_ptSq - 1: n_pos + n_ptSq] = 0
        Sig[n_pos + n_ptSq - 1, n_pos + n_ptSq - 1] = 1e6
        Sig[n_pos + n_ptSq, n_pos + n_ptSq] = 1e6

        mu[pt_idSq - 1 + n_pos: n_pos + n_ptSq - 1] = mu[pt_idSq + 1 + n_pos: n_pos + n_ptSq]
        mu[n_pos + n_ptSq - 1: n_pos + n_ptSq] = 0

    if ln_id > 0:
        Sig[ln_idSq - 1 + n_pos + n_ptSq: n_pos + n_ptSq + n_lnSq - 1, :] = Sig[ln_idSq + 1 + n_pos + n_ptSq: n_pos + n_ptSq + n_lnSq, :]
        Sig[:, ln_idSq - 1 + n_pos + n_ptSq: n_pos + n_ptSq + n_lnSq - 1] = Sig[:, ln_idSq + 1 + n_pos + n_ptSq: n_pos + n_ptSq + n_lnSq]
        Sig[n_pos + n_ptSq + n_lnSq - 1: n_pos + n_ptSq + n_lnSq, :] = 0
        Sig[:, n_pos + n_ptSq + n_lnSq - 1: n_pos + n_ptSq + n_lnSq] = 0
        Sig[n_pos + n_ptSq + n_lnSq, n_pos + n_ptSq + n_lnSq] = 1e6
        Sig[n_pos + n_ptSq + n_lnSq - 1, n_pos + n_ptSq + n_lnSq - 1] = 1e6

        mu[ln_idSq - 1 + n_pos + n_ptSq: n_pos + n_ptSq + n_lnSq - 1] = mu[ln_idSq + 1 + n_pos + n_ptSq: n_pos + n_ptSq + n_lnSq]
        mu[n_pos + n_ptSq + n_lnSq - 1: n_pos + n_ptSq + n_lnSq] = 0

    return mu, Sig

def modifyPtLMs(i, hist_i, countarr, expN, val_pt, ind_pt, mu, Sig, N_pts):
    # Initialize variables
    if ind_pt > 0:
        delt_i = i - hist_i[ind_pt]
        if delt_i < 30 or hist_i[ind_pt] == 0:
            countarr[ind_pt] += 1
        elif delt_i > 200 and countarr[ind_pt] < val_pt:
            countarr[ind_pt] = 0
            mu, Sig = modify_mu_sig(mu, Sig, [ind_pt, 0], expN)
            N_pts -= 1
            i = 0
        hist_i[ind_pt] = i

    return mu, Sig, N_pts, countarr, hist_i

def modifyLinLMs(i, hist_i, countarr, expN, val_ln, ind_ln, mu, Sig, N_line, max_ind, z, visLine_x, visLine_y):
    if ind_ln > 0:
        index_ln = expN[0] + ind_ln

        delt_i = i - hist_i[index_ln]

        if delt_i < 100 or hist_i[index_ln] == 0 or countarr[index_ln] > 50:
            countarr[index_ln] += 1
        elif delt_i > 130 and countarr[index_ln] < val_ln:
            countarr[index_ln] = 0
            countarr[index_ln:-1] = countarr[index_ln + 1:]
            countarr[-1] = 0
            mu, Sig = modify_mu_sig(mu, Sig, [0, ind_ln], expN)
            visLine_x[ind_ln:-1, :] = visLine_x[ind_ln + 1:, :]
            visLine_x[-1, :] = 0
            visLine_y[ind_ln:-1, :] = visLine_y[ind_ln + 1:, :]
            visLine_y[-1, :] = 0
            N_line -= 1
            i = 0

        if max_ind == 2:
            for k in range(1, expN[0] + 1):
                if countarr[k] > 20:
                    m, c = rth2mc2(mu, z[0], z[1])
                    D = pt2line([m, c], mu[2*k + 2: 2*k + 3])
                    if D < 50:
                        mu, Sig = modify_mu_sig(mu, Sig, [0, ind_ln], expN)
                        N_line -= 1
                        visLine_x[ind_ln:-1, :] = visLine_x[ind_ln + 1:, :]
                        visLine_x[-1, :] = 0
                        visLine_y[ind_ln:-1, :] = visLine_y[ind_ln + 1:, :]
                        visLine_y[-1, :] = 0
                        i = 0
                        countarr[index_ln] = 0
                        countarr[index_ln:-1] = countarr[index_ln + 1:]
                        countarr[-1] = 0
                        break

                if countarr[k] == 0:
                    break

        hist_i[index_ln] = i

    return mu, Sig, N_line, countarr, hist_i, visLine_x, visLine_y

# Update the limits of the SLAM plot
def updateLimsX(k, visLine_x, new_x):
    if visLine_x[k, 0] == 0:
        visLine_x[k, 0] = new_x[0]
        visLine_x[k, 1] = new_x[1]
    # elif new_x[0] < new_x[1]:
    else:
        if visLine_x[k, 0] > new_x[0]:
            visLine_x[k, 0] = new_x[0]
        if visLine_x[k, 1] < new_x[1]:
            visLine_x[k, 1] = new_x[1]
    # else:  # new_x[0] > new_x[1]
    #     pdb.set_trace()
    #     if visLine_x[k, 0] > new_x[1]:
    #         visLine_x[k, 0] = new_x[1]
    #     if visLine_x[k, 1] < new_x[0]:
    #         visLine_x[k, 1] = new_x[0]
    return visLine_x
def updateLimsY(k, visLine_y, new_y):
    if visLine_y[k, 0] == 0:
        visLine_y[k, 0] = new_y[0]
        visLine_y[k, 1] = new_y[1]
    elif visLine_y[k, 0] < visLine_y[k, 1]:
        if visLine_y[k, 0] > new_y[0]:
            visLine_y[k, 0] = new_y[0]
        if visLine_y[k, 1] < new_y[1]:
            visLine_y[k, 1] = new_y[1]
    else:  # new_y[0] > new_y[1]
        if visLine_y[k, 0] < new_y[0]:
            visLine_y[k, 0] = new_y[0]
        if visLine_y[k, 1] > new_y[1]:
            visLine_y[k, 1] = new_y[1]
    return visLine_y

# Checks whether the center is isolated from the rest
def chkIsolated(points, center, thresh1, thresh2, offset=0):
    iso = False
    center = center.reshape(1, -1)
    # draws two circles with different radii and check the number of inliers
    # thresh1 and thresh2 are circle radii
    # if both inlier counts are same, the center is isolated
    L = points.shape[0] # number of points
    dist = np.zeros(L) # stores the distance values

    for p in range(L): # calculates the distance
        dist[p] = np.linalg.norm(points[p] - center)

    # pdb.set_trace()
    # if sum(dist < thresh1) == sum(dist < thresh2) > 0: # if the number of points are same inside both circles,
    #     iso = True # the point is isolated

    sum_thresh1 = sum(dist < thresh1)
    sum_thresh2 = sum(dist < thresh2)
    if sum_thresh1 > 0 and (sum_thresh2 - sum_thresh1) <= offset:
        iso = True # the point is isolated
        print('Isolated: ', sum(dist < thresh2), sum(dist < thresh1))

    return iso

# Find nearby lines
def findNearbyLines(pose, z, expN, visLine_x, visLine_y):
    D = []
    for k in range(expN):
        pt1 = [visLine_x[k, 0], visLine_y[k, 0]]
        pt2 = [visLine_x[k, 1], visLine_y[k, 1]]

        pt0 = [pose[0]+z[0]*np.cos(z[1]+pose[2]), pose[1]+z[0]*np.sin(z[1]+pose[2])]

        pp = project_point_to_line(pt0, pt1, pt2)
        if pt1[0] < pp[0] < pt2[0] or pt2[0] < pp[0] < pt1[0]: # if the projection point is within the line segment
            d1 = pt2pt_sq(pp, pt0) # squared distance between the projection point and the robot observation
            d2 = d1

        else:
            d1 = pt2pt_sq(pt1, pt0)
            d2 = pt2pt_sq(pt2, pt0)

        D.append(np.sqrt(min(d1, d2)[0]))
    # print('D_min: ', D)
    min_index, D_min = find_min_index(D)
    return D, D_min, min_index

# Find the squared distance between two lines
def pt2pt_sq(pt1, pt2):
    return (pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2

# Find the minimum value and its index
def find_min_index(lst):
    min_value = min(lst)
    min_index = lst.index(min_value) 
    return min_index, min_value  # returns the index and the value

def pts2mc(pt1, pt2):
    m = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
    c = pt1[1] - m * pt1[0]
    return m, c

def rotate_points(points, theta_rad):
    # theta is in radians
    # points are numpy array of shape (n, 2)

    # Define rotation matrix
    rotation_matrix = np.array([[np.cos(theta_rad), -np.sin(theta_rad)],
                                 [np.sin(theta_rad), np.cos(theta_rad)]])

    # Apply rotation
    rotated_points = np.dot(points, rotation_matrix)

    return rotated_points

def project_point_to_line(P, A, B):
    x1, y1 = P
    x2, y2 = A
    x3, y3 = B

    # Vector AB
    ABx = x3 - x2
    ABy = y3 - y2

    # Vector AP
    APx = x1 - x2
    APy = y1 - y2

    # Dot product of AP and AB
    dot_product = APx * ABx + APy * ABy

    # Magnitude squared of AB
    magnitude_squared = ABx**2 + ABy**2

    # Projection scalar t
    t = dot_product / magnitude_squared

    # Projection point coordinates
    Px = x2 + t * ABx
    Py = y2 + t * ABy

    return [Px, Py]

def get_indexes_below_threshold(input_list, threshold):
    return [index for index, value in enumerate(input_list) if value < threshold]

def sort_two_lists(list1, list2):
    """
    Sorts list1 in ascending order and rearranges list2
    so that the corresponding elements follow the sorted order of list1.

    Parameters:
    list1 (list): The list to be sorted.
    list2 (list): The list to be rearranged.

    Returns:
    tuple: A tuple containing the sorted list1 and rearranged list2.
    """
    # Sort the combined list based on the first list's elements
    sorted_combined = sorted(zip(list1, list2), reverse=False) # reverse=True for descending order

    # Unzip the sorted combined list
    _, sorted_list2 = zip(*sorted_combined)

    # Convert the tuples back to lists
    return list(sorted_list2)
