# ROS2 Workspace for UWB Radar SLAM

This is the ROS2 workspace of the proposed UWB Radar SLAM.
Clone the repo and build it.

## Instructions to reproduce the experimental results in the paper.

1. Go to DOI Link: [https://dx.doi.org/10.21227/shx8-gw47] and find the datasets (there are 4 datasets from the experiments mentioned in the paper).

2. Select the required dataset (e.g. `experiment_1_data`) and find the `dataset_mph6_mpp5.mat` file.

3. Copy the `dataset_mph6_mpp5.mat` to the `dataset` directory in the ROS2 workspace.

4. Edit the `params.yaml` files in the following directory:
``` ~/paper3_ws/install/lp_slam/share/lp_slam/config ```
