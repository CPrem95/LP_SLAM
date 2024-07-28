import numpy as np
import scipy.io
import pdb
import matplotlib.pyplot as plt
import math

plt.ion()
def set_limits(ax, data, odom):
    tmp_xmin=min(data[0]); tmp_xmax=max(data[0])
    tmp_ymin=min(data[1]); tmp_ymax=max(data[1])
    tmp_xmin2 = min(odom[:, 0]); tmp_xmax2 = max(odom[:, 0])
    tmp_ymin2 = min(odom[:, 1]); tmp_ymax2 = max(odom[:, 1])

    xmin = min(tmp_xmin, tmp_xmin2); xmax = max(tmp_xmax, tmp_xmax2)
    ymin = min(tmp_ymin, tmp_ymin2); ymax = max(tmp_ymax, tmp_ymax2)

    ax.set_xlim(xmin - 3000, xmax + 3000)
    ax.set_ylim(ymin - 3000, ymax + 3000)

exp_line_landm = 50
exp_pt_landm = 50
gt_dataset_name = "gt_data"         # The ground truth data >>> odom and SLAM_toolbox gnd trth
est_dataset_name = "final_results"  # Final results from LP SLAM >>> 
align_dataset_name = "aligned_est"  # Final results from LP SLAM >>>

gt_data = scipy.io.loadmat('/home/arms/Documents/' + str(gt_dataset_name) + ".mat")
est_data = scipy.io.loadmat('/home/arms/Documents/' + str(est_dataset_name) + ".mat")
align_data = scipy.io.loadmat('/home/arms/Documents/' + str(align_dataset_name) + ".mat")

mu = est_data['mu']
N_line = est_data['N_line'][0][0]
N_pts = est_data['N_pts'][0][0]
visLine_x = est_data['visLine_x']
visLine_y = est_data['visLine_y']
odom = est_data['odom']
x_history = est_data['x_history'][0]
y_history = est_data['y_history'][0]

gt_x = gt_data['gt_x']
gt_y = gt_data['gt_y']

############################################################################################################
## SLAM Plot
############################################################################################################
# Figure for final estimated lines and points plot
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_aspect('equal', adjustable='box')
plt.grid(linestyle="--", color='black', alpha=0.3)
ax.set_xlabel('x [mm]')
ax.set_ylabel('y [mm]')
ax.set_title('Final SLAM output')
plt.show()

# Estimated path plot
raw_odom, = ax.plot([], [], linewidth = 2, c='b', linestyle= '--', label='Odometry')
fig.canvas.draw()
est_path, = ax.plot([], [], linewidth = 2, c='r', label='Estimated path')
fig.canvas.draw()
# Estimated pose
est_pose = ax.scatter([], [], s=30, c='navy', marker='o', label='Current pose')
fig.canvas.draw()
# Estimated point landmarks plot
est_pt_lms = ax.scatter([], [], s=30,  edgecolors='magenta', facecolors='none', marker='s', label='Estimated point landmarks')
fig.canvas.draw()
# Estimated line landmarks plot
est_ln_lms = []
for i in range(exp_line_landm):
    tmp_plt_ln_all, = ax.plot([], [], c='darkorange', linewidth = 3, linestyle= '-', label='Estimated line landmarks')
    est_ln_lms.append(tmp_plt_ln_all)
    fig.canvas.draw()

# Legend
fig.legend(handles=[raw_odom, est_path, est_pt_lms, tmp_plt_ln_all], loc='upper right')
fig.canvas.flush_events()
    

# Plot the raw odometry
raw_odom.set_xdata(odom[:, 0])
raw_odom.set_ydata(odom[:, 1])
print("odom added")
# Plot the estimated path
est_path.set_xdata(x_history)
est_path.set_ydata(y_history)
print("estimated path added")
# Plot the estimated landmarks
est_pt_lms.set_offsets(mu[3:3+2*N_pts].reshape(-1, 2))
est_pose.set_offsets([mu[0][0], mu[1][0]]) # estimated pose

# Adjust the axis limits
set_limits(ax, [x_history, y_history], odom)
fig.canvas.draw()

# Plot the estimated landmarks
# Lines
lnstart = exp_pt_landm*2 + 3
pi = math.pi
for lin_i in range (N_line):
    print("adding line: ", lin_i)
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
print("lines added")
# Points
est_pt_lms.set_offsets(mu[3:3+2*N_pts].reshape(-1, 2))
print('points added')
fig.canvas.draw()

fig.canvas.flush_events()
pdb.set_trace()
print("Adjust the plot size and Press 'c' to save the plot")
# Save the figure with the grid >>> raw odometry and estimated path
plt.savefig('SLAM_plot1.png', dpi=600) 
print("Plot1 saved")
# Save the figure with high resolution
plt.grid(False)
plt.savefig('SLAM_plot2.png', dpi=600)
print("Plot2 saved")
pdb.set_trace()

# Figure for final estimated lines and points plot
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_aspect('equal', adjustable='box')
ax.set_xlabel('x [mm]')
ax.set_ylabel('y [mm]')
ax.set_title('Final SLAM output')
plt.show()

# Estimated path plot
raw_odom, = ax.plot([], [], linewidth = 2, c='b', linestyle= '--', label='Odometry')
fig.canvas.draw()
est_path, = ax.plot([], [], linewidth = 2, c='r', label='Estimated path (ours)')
fig.canvas.draw()
# Estimated pose
est_pose = ax.scatter([], [], s=30, c='navy', marker='o', label='Current pose')
fig.canvas.draw()
# Estimated point landmarks plot
est_pt_lms = ax.scatter([], [], s=30,  edgecolors='magenta', facecolors='none', marker='s', label='Estimated point landmarks')
fig.canvas.draw()
# Estimated line landmarks plot
est_ln_lms = []
for i in range(exp_line_landm):
    tmp_plt_ln_all, = ax.plot([], [], c='darkorange', linewidth = 3, linestyle= '-', label='Estimated line landmarks')
    est_ln_lms.append(tmp_plt_ln_all)
    fig.canvas.draw()
fig.canvas.flush_events()

# Adjust the axis limits
set_limits(ax, [x_history, y_history], odom)
fig.canvas.draw()

# for lin_i in range (N_line):
#     print("adding line: ", lin_i)
#     line_r = mu[2*lin_i + lnstart]
#     line_th = mu[2*lin_i + lnstart + 1]

#     line_m = math.tan(pi/2 + line_th)
#     line_c = line_r/math.sin(line_th)

#     if abs(abs(line_th) - pi/2) < pi/4:
#         x = visLine_x[lin_i, :]
#         y = line_m*x + line_c
#     else:
#         y = visLine_y[lin_i, :]
#         x = (y - line_c)/line_m

#     est_ln_lms[lin_i].set_xdata(x)
#     est_ln_lms[lin_i].set_ydata(y)
#     fig.canvas.draw()
# print("lines added")
# # Points
# est_pt_lms.set_offsets(mu[3:3+2*N_pts].reshape(-1, 2))
# print('points added')
# fig.canvas.draw()


gt_path, = ax.plot(gt_x[0], gt_y[0], linewidth = 2, c='g', label='Ground truth')
fig.canvas.draw()
# Legend
fig.legend(handles=[raw_odom, est_path, gt_path, est_pt_lms, tmp_plt_ln_all], loc='upper right')
fig.canvas.flush_events()
pdb.set_trace()
plt.savefig('SLAM_plot3.png', dpi=600)
print("Plot3 saved")

