# Used to evaluate the performance of the LP SLAM algorithm by comapring the estimated poses with the ground truth poses.
# Use map2base.py to save the ground truth data and the estimated data from the LP SLAM algorithm.
# Use lp_node_mat.py to save the estimated data from the LP SLAM algorithm.

import open3d as o3d
import numpy as np
import scipy.io
import pdb
import matplotlib.pyplot as plt
from scipy.io import savemat

gt_dataset_name = "gt_data"         # The ground truth data >>> odom and SLAM_toolbox gnd trth
est_dataset_name = "final_results"  # Final results from LP SLAM >>> 

gt_data = scipy.io.loadmat('/home/arms/Documents/' + str(gt_dataset_name) + ".mat")
est_data = scipy.io.loadmat('/home/arms/Documents/' + str(est_dataset_name) + ".mat")
                              
gt_x = gt_data['gt_x']
gt_y = gt_data['gt_y']
gt_odom_x = gt_data['gt_x']
gt_odom_y = gt_data['gt_y']

est_x = est_data['x_history']
est_y = est_data['y_history']

gt_poses = np.array([gt_x[0], gt_y[0]]).T
est_poses = np.array([est_x[0], est_y[0]]).T

gt_odom = np.array([gt_odom_x[0], gt_odom_y[0]]).T
est_odom = est_data['odom'][:,0:2]

# Create example 2D point clouds
moving_points = est_poses   # moving points
fixed_points = gt_poses    # Fixed ground truth points

# Convert to 3D by adding a zero z-coordinate
moving_points_3d = np.c_[moving_points, np.zeros(moving_points.shape[0])]
fixed_points_3d = np.c_[fixed_points, np.zeros(fixed_points.shape[0])]

# Create Open3D point clouds
moving = o3d.geometry.PointCloud()
moving.points = o3d.utility.Vector3dVector(moving_points_3d)

fixed = o3d.geometry.PointCloud()
fixed.points = o3d.utility.Vector3dVector(fixed_points_3d)

# Set the color of the point clouds
moving.paint_uniform_color([1, 0, 0])  # Red
fixed.paint_uniform_color([0, 0, 1])   # Blue

# Visualize the moving and fixed point clouds
o3d.visualization.draw_geometries([moving, fixed], window_name="moving and fixed", width=800, height=600)

# Initial alignment using a rough transformation
trans_init = np.eye(4)
moving.transform(trans_init)

# Apply ICP
threshold = 500  # Distance threshold for point matching
icp_result = o3d.pipelines.registration.registration_icp(
    moving, fixed, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
) # Point-to-point ICP

# Get the transformation matrix
transformation = icp_result.transformation
print("ICP Transformation Matrix:")
print(transformation)

# Transform the moving point cloud using the ICP result
moving.transform(transformation)

# Visualize the aligned point clouds
o3d.visualization.draw_geometries([moving, fixed], window_name="Aligned Point Clouds", width=800, height=600)

## RMSE CALCULATIONS
#--------------------------------------------------------------------------------
def nearest_points(A, B):
    # For ICP
    # A: moving points
    # B: fixed points
    # nA < nB

    # Initialize an array to store the nearest points in B for each point in A
    nearest_points = []
    # Initialize an array to store the indices of the nearest points in B for each point in A
    nearest_indices = []

    # Iterate over each point in A
    for point_a in A:
        # Compute the Euclidean distance from point_a to all points in B
        distances = np.linalg.norm(B - point_a, axis=1)
        
        # Find the index of the minimum distance
        min_index = np.argmin(distances)

        # Append the index of the nearest point in B to the list
        nearest_indices.append(min_index)
        
        # Append the nearest point in B to the list
        nearest_points.append(B[min_index])
    
    return nearest_points, nearest_indices

# Resample the moving point cloud to match the number of fixed points
_, nearest_indices = nearest_points(moving_points, fixed_points)
pdb.set_trace()
resampled_fixed_points = fixed_points[nearest_indices]

# Calculate RMSE
transformed_moving_points = np.asarray(moving.points)[:, :2]  # Ignore the z-coordinate
resampled_fixed_points = fixed_points[nearest_indices]

savemat("aligned_est.mat", {"aligned_est": transformed_moving_points})

# Compute RMSE
rmse = np.sqrt(np.mean((transformed_moving_points - resampled_fixed_points) ** 2))
print(f"RMSE: {rmse:.4f} mm")


# Plot using matplotlib with a legend
plt.figure()
plt.scatter(moving_points[:, 0], moving_points[:, 1], color='red', label='Estimated Points (LP SLAM)')
plt.scatter(fixed_points[:, 0], fixed_points[:, 1], color='green', label='Ground Truth (SLAM Toolbox)')

# After ICP transformation
plt.scatter(transformed_moving_points[:, 0], transformed_moving_points[:, 1], color='blue', marker='x', label='Estimated Points (transformed)')

plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Point Clouds with Legend')
plt.show()