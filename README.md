# ROS2 Workspace for UWB Radar SLAM

This is the ROS2 workspace of the proposed UWB Radar SLAM.
Clone the repo and build it.

## Instructions to reproduce the experimental results in the paper.

1. IEEE Dataport: Go to DOI Link [https://dx.doi.org/10.21227/shx8-gw47] and find the datasets (there are 4 datasets from the experiments mentioned in the paper).

2. Select the required dataset (e.g. `experiment_1_data`) and find the `dataset_mph6_mpp5.mat` file.

3. Copy the `dataset_mph6_mpp5.mat` to the `dataset` directory in the ROS2 workspace. Rename the file to `dataset.mat`.

4. Edit the `params.yaml` files in the following directory:
```
~/<workspace_name>/install/lp_slam/share/lp_slam/config
```
`params.yaml` includes the parameters used in the Vision-denied experiments (i.e. exp_1 and 2). Therefore, use them in the datasets #1 and #2 from the IEEE dataport.
`params0.yaml` includes the parameters used in the remaining experiments (i.e. exp_3 and 4). Therefore, use them in the datasets #3 and #4 from the IEEE dataport. Hence, you may just rename the  `params0.yaml` to `params.yaml` to avoid hassle.

5. Open a terminal and use the following command to visualize the map building results as in the multimedia file.
```
ros2 run lp_slam lp_node_mat
```

## Instructions to generate a new `dataset.mat` file.

mph6 and mpp5 in `dataset_mph6_mpp5.mat` refers to the maximum peak height of 6e-3 and maximum peak prominence of 5e-3 during peak finding from the UWB radar observations.
We have use the same `mph` and  `mpp` during our experiments.
 
If you want to change those parameters and test, you should follow the fillowing steps.
