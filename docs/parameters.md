# Important parameters

!!! info
    See the [sensor setup guide](https://github.com/koide3/glim/wiki/Sensor-setup-guide) for configurations of popular sensors (including Livox MID360 and Azure Kinect).

!!! info
    See the Configuration files section in [Getting started](quickstart.md) to change the location of configuration files.

## ROS-related (config_ros.json)

- **acc_scale** (default 1.0) : Linear acceleration scaling factor. Set this to 9.80665 if the unit of IMU linear acceleration is [g] but not [m/s^2] (e.g., Livox LiDARs). 
- **(imu|points|image)_topics** : Input data topics.

## Sensor configuration (config_sensors.json)

- **T_lidar_imu** : Transformation from the IMU frame to the LiDAR frame (See [notation](extend.md##Notation)). When the IMU is at rest and the IMU z-axis points upwards, linear acceleration vector should be around [0, 0, +9.81] (See also [ROS REP 145](https://www.ros.org/reps/rep-0145.html) and [FAQ](faq.md)).

## Preprocessing (config_preprocess.json)

- **random_downsample_target** (default 10000 points): Target number of points for downsampling. Reducing the target number of points (e.g., to 5000) makes estimation significantly faster.

- **k_correspondences** (default 10 points): The number of neighboring points used for covariance estimation. For LiDARs with sparse scan patterns (e.g., Velodyne VLP16), increase this value to 15 ~ 30 to avoid degeneration of covariance matrices.

!!! note
    To see if estimated covariances are fine, change ```color_mode``` in the standard viewer to ```NORMAL```. If point colors are uniform on flat planes, covariances should be ok.

## GPU-based LiDAR-IMU Odometry Estimation (config_odometry.json)

- **voxel_resolution** (default 0.25 m) : Base VGICP voxel resolution. Use a small value for indoor environments (e.g., 0.1 ~ 0.25 m).
- **voxelmap_levels** (default 2 levels): Multi resolution voxel levels. Increasing this parameter makes estimation robust to large displacement.
- **max_num_keyframes** (default 15 keyframes): Maximum number of keyframes. Increasing this parameter reduces odometry estimation drift.
- **keyframe_update_strategy** (default OVERLAP): *"OVERLAP"*, *"DISPLACEMENT"*, or *"ENTROPY"*. 
    - *"OVERLAP"* uses an overlap-metric-based keyframe management strategy that can adaptively deal with many environments (indoors and outdoors). Increasing **keyframe_max_overlap** makes keyframe insertion more frequent and robust to dynamic situations.
    - *"DISPLACEMENT"* uses the conventional displacement-based keyframe management that is more intuitive to tune. Change **keyframe_delta_(trans|rot)** to tune the keyframe insertion frequency.
    - *"ENTROPY"* uses an entropy-based keyframe management. This strategy is often difficult to tune and is not recommended.

## CPU-based LiDAR-IMU Odometry Estimation (config_odometry_cpu.json)

- **registration_type** (default GICP) : Either of *"GICP"* or *"VGICP"*.
    - *"GICP"* uses iVox-based GICP scan matching that is accurate and robust in many cases.
        - **ivox_resolution** (default 0.5 m) : Resolution of iVox voxels used for GICP scan matching. This parameter also controls the maximum corresponding distance and should be set to a large value in outdoor environments (e.g., 1.0 m).

    - *"VGICP"* uses voxelized GICP scan matching that is faster but requires tuning **vgicp_resolution** parameter for good estimation in indoor environments.
        - **vgicp_resolution** (default 0.5 m) : Resolution of VGICP voxels used for VGICP scan matching. Use a small value for indoor environments (e.g., 0.25 ~ 0.5 m) and a large value for outdoor environments (0.5 ~ 2.0 m).

## LiDAR-only Odometry Estimation (config_odometry_ct.json)

- **max_correspondence_distance** (default 2.0 m) : Maximum corresponding distance for scan matching. 


## Global Optimization (config_sub_mapping.json & config_global_mapping.json)

### Sub mapping
- **enable_optimization** (default true) : In environments where the odometry estimation is sufficiently robust and accurate, you can set this false to disable submap optimization and save the processing cost.
- **keyframe-related params** : These parameters control the keyframe creation in sub mapping. See GPU-based LiDAR-IMU odometry params for details.

### Global mapping
- **min_implicit_loop_overlap** (default 0.2) : Minimum overlap rate to create registration error factor.
- **enable_loop_verification** (default false) : Register each revisit (a run of overlapping past submaps that are not the ones just traversed) coarse-to-fine before creating registration error factors. If the current estimate is outside the convergence basin of the factors, a BetweenFactor from the registration pulls it in once later submaps agree with it, and is replaced with registration error factors when the revisit is in the basin. Without it, an out-of-basin revisit gets registration error factors at a wrong local minimum.
- **loop_coarse_levels** (default 4) : Number of coarse-to-fine registration stages. Stage k uses a correspondence distance of epcd_max_correspondence_distance x 2^k.
- **loop_consistency_submaps** (default 2) : Number of agreeing registrations required before a loop correction is inserted.
- **epcd_eager_coreset_extraction** (default false) : Extract the EPCD coreset right after full sampling instead of at the next linearization. The linearization results are the same; it releases the sampled features, which global mapping would otherwise keep for most factors.

### Common parameters for sub and global mapping
- **enable_imu** (default true) : Must be false if the LiDAR-only odometry estimation is used.
- **registration_error_factor_type** (default "VGICP_GPU") : Registration error computation type. Must be either of *"VGICP"* or *"VGICP_GPU"*.
- **random_sampling_rate** (default 1.0) : Random sampling rate for points used for registration error computation. With the GPU implementation, you can use a large random sampling rate (e.g., 1.0 = disabling random sampling) to perform full global registration error minimization.
- **(submap|keyframe)_voxel_resolution** (default 0.5 m) : Base voxel resolution. Set a small value (e.g., 0.15 ~ 0.25 m) for indoor environments.
- **(submap|keyframe)_voxelmap_levels** (default 2 levels) : Multi resolution voxel levels. Set this param to 2 or 3 for better convergence.
