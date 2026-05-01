# FAST-LIO-SAM-G

A LiDAR-inertial SLAM system that integrates **FAST-LIO2** as the high-frequency frontend with a **LIO-SAM-style** factor graph backend for global optimization, and further provides a **GridMap** projection pipeline for navigation, supporting **RoboSense LiDARs**, **Unilidar LiDARs**, and compatible with both **ROS1** and **ROS2**. The repository is based on [FAST-LIO-SAM](https://github.com/RightTr/FAST-LIO-SAM.git).

## 🧩 Contributions

* A SLAM system that integrates FAST-LIO2 with a LIO-SAM-style factor graph backend.

* Manual initial pose setting for relocalization

* Stationary detection and adaptive weight handling between LiDAR update scans and ZUPT

* Online GridMap projection from optimized global point cloud to 2D `OccupancyGrid`

## 🛠️ Prerequisites

### Dependency

* [gtsam](https://gtsam.org/get_started/) (Georgia Tech Smoothing and Mapping library)

```bash
# Ubuntu 20.04
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
# Ubuntu 22.04
sudo add-apt-repository ppa:borglab/gtsam-release-4.1

sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### ROS1 Build

```bash
mkdir fastlio_sam_ws
cd fastlio_sam_ws

mkdir src && cd src
git clone https://github.com/RightTr/FAST-LIO-SAM.git

cd src/FAST-LIO-SAM
git submodule update --init --recursive

# ROS1 build
./build.sh ROS1

# ROS2 build
./build.sh humble
```

## 🚀 Usage

### LIO-SAM-style Backend

```bash
cd fastlio_ws
source devel/setup.bash
# e.g.
roslaunch fast_lio_sam_g sam_airy.launch
```

### GridMap Pipeline

For navigation-oriented usage, the recommended entry is the GridMap pipeline. It keeps the optimized 3D map and the exported 2D navigation map in the same SLAM system instead of running another online 2D SLAM node.

```bash
# ROS2
cd fastlio_ws
source install/setup.bash
ros2 launch fast_lio_sam_g gridmap_mid360.launch.py
```

### Relocalization

The modified system supports relocalization from RViz `2D Pose Estimate`. The pose is received on `/initialpose`, then registered against the loaded 3D keyframe clouds before the filter state is reset.

Run relocalization mode:

```bash
# e.g.
# ROS1
cd fastlio_ws
source devel/setup.bash
roslaunch fast_lio_sam_g reloc_mid360.launch
# Use RViz 2D Pose Estimate, which publishes geometry_msgs::PoseWithCovarianceStamped to /initialpose

# ROS2
cd fastlio_ws
source install/setup.bash
ros2 launch fast_lio_sam_g reloc_mid360.launch.py
# Use RViz 2D Pose Estimate, which publishes geometry_msgs::msg::PoseWithCovarianceStamped to /initialpose
```

## Stationary detection and adaptive weight handling

The system will adjust the confidence of the ZUPT and LiDAR updates based on the detected motion state, using **accelerometer and gyroscope variances** as well as **the EMA of velocity**. When the system detects a stationary state, it will increase the confidence of the ZUPT update and decrease the confidence of the LiDAR update, and vice versa when in motion.

Check the related parameters in the .yaml files.

```yaml
zupt:
    use_zupt: true                    # enable adaptive zero velocity update
    zupt_acc_var_threshold:  0.0001   # (m/s²)² per-axis acc variance → full-confidence threshold
    zupt_gyro_var_threshold: 0.00001  # (rad/s)² per-axis gyro variance → full-confidence threshold

    # Static confidence: exp(-3 * max_normalized_variance), in [0,1]
    zupt_confidence_min: 0.05         # below this no ZUPT is applied

    # Dynamic ZUPT weight: R = clamp(zupt_r_min / eff_confidence, zupt_r_max)
    zupt_r_min: 1.0e-5                # ZUPT measurement noise at full confidence
    zupt_r_max: 1.0                   # ZUPT measurement noise cap (≈ no constraint)

    # Dynamic LiDAR weight: lidar_cov = LASER_POINT_COV * (1 + scale*conf) * max(1, residual/ref)
    lidar_cov_static_scale: 5.0       # LiDAR cov multiplier at full confidence
    lidar_residual_ref: 0.05          # (m) reference residual; above this LiDAR trust reduces

    # Covariance inflation (lock-in prevention)
    cov_inflate_start: 200            # IMU steps of continuous static before inflation kicks in
    cov_inflate_pos:   1.0e-7         # per-step position covariance inflation
    cov_inflate_rot:   1.0e-8         # per-step rotation covariance inflation
```

### GridMap

The GridMap node projects the optimized global point cloud to `/map` as a 2D `OccupancyGrid`.

The current design assumes:

* the global point cloud comes from the backend-optimized map

* the 2D navigation map should stay consistent with the 3D map

### Export / Import Keyframe Map

Set `keyframe_map/map_path` to a directory path. When LIO-SAM backend is enabled, set `keyframe_map/map_save` to `true` to save the keyframe map during shutdown. Set `keyframe_map/map_load` to `true` to load the saved keyframe map during startup. The map is stored under `KEYFRAMES/`, with `2d/` used for grid mapping and online scan publishing and `3d/` used for relocalization.

## 📝 TODO List

* [x] GridMap projection pipeline
* [ ] GridMap ROS1 adaptation
* [ ] Incremental GridMap update optimization

## 📚 Related Works

[FAST-LIO2: Fast Direct LiDAR-inertial Odometry](doc/Fast_LIO_2.pdf)

[FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)

[FAST-LIO official repository](https://github.com/hku-mars/FAST_LIO.git)

[fast_lio_sam_g](https://github.com/kahowang/fast_lio_sam_g.git)

[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM.git)

[robosense_fast_lio](https://github.com/RuanJY/robosense_fast_lio.git)

[point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar.git)
