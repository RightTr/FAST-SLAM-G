# FAST-LIO-SAM

A LiDAR-inertial SLAM system that integrates **FAST-LIO2** as the high-frequency frontend with a **LIO-SAM-style** factor graph backend for global optimization, supporting **RoboSense series LiDARs**, **Unilidar series LiDARs**, and compatible with both **ROS1** and **ROS2**.

## 🧩 Contributions

* A SLAM system that integrates FAST-LIO2 with a LIO-SAM-style factor graph backend.

* ROS1 and ROS2 adaptation

* High-frequency odometry via IMU propagation between LiDAR scans

* Manual initial pose setting for relocalization

* ZUPT detection and handling

* Support for RoboSense series LiDARs, Unilidar series LiDARs

## 🛠️ Prerequisites

### Dependency

* [gtsam](https://gtsam.org/get_started/) (Georgia Tech Smoothing and Mapping library)

```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### ROS1 Build

```bash
mkdir fastlio_sam_ws
cd fastlio_sam_ws

mkdir src && cd src
git clone https://github.com/RightTr/FAST-LIO-SAM.git

# Clone the FAST-LIO interfaces package for ROS1
git clone https://github.com/RightTr/fast_lio_interfaces.git

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
roslaunch fast_lio_sam sam_airy.launch
```

### Relocalization

The modified system supports relocalization using manually set odometry poses. Once odometry poses are published to the */reloc_topic* (according to the following .yaml file), the system will reset the system and the initial pose according to your input.

Run relocalization mode:

```bash
# e.g.
# ROS1
cd fastlio_ws 
source devel/setup.bash
roslaunch fast_lio_sam reloc_mid360.launch
# Publish geometry_msgs::PoseStamped to the /reloc_topic

# ROS2
cd fastlio_ws 
source install/setup.bash
ros2 launch fast_lio_sam reloc_mid360.launch.py
# Publish geometry_msgs::msg::PoseStamped to the /reloc_topic
```

## ZUPT Detection and Handling

Check the related parameters in the .yaml files.

```yaml
zupt:
    use_zupt: false              # enable zero velocity update
    zupt_acc_norm_threshold: 0.30 # (m/s²) acceleration norm threshold for ZUPT
    zupt_gyro_threshold: 0.05    # (rad/s) gyroscope threshold for ZUPT
```

### High frequency odometry via IMU propagation between LiDAR scans

Subscribe the topic */OdometryHighFreq* to receive high frequency odometry output via IMU propagation between LiDAR scans.

### Extended LiDAR support

Now, FAST-LIO supports tracking and mapping using the RoboSense LiDARs (e.g., RoboSense Airy) and Unilidar LiDARs (e.g., Unilidar L1). Check the related files in ./config and ./launch folder.

```bash
# e.g.
roslaunch fast_lio_sam mapping_airy.launch
```

## 📝 TODO List

- [x] Full ROS2 adaptation
- [ ] ROS2 Adaptation Test

## 📚 Related Works

[FAST-LIO2: Fast Direct LiDAR-inertial Odometry](doc/Fast_LIO_2.pdf)

[FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)

[FAST-LIO official repository](https://github.com/hku-mars/FAST_LIO.git)

[FAST_LIO_SAM](https://github.com/kahowang/FAST_LIO_SAM.git)

[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM.git)

[robosense_fast_lio](https://github.com/RuanJY/robosense_fast_lio.git)

[point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar.git)
