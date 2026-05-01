#ifndef MAP_OPTIMIZATION_H
#define MAP_OPTIMIZATION_H

#include <string>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

using PointTypePose = PointXYZIRPYT;

extern float transformTobeMapped[6];
extern Eigen::Vector3d translationLidarToIMU;
extern Eigen::Matrix3d rotationLidarToIMU;

void MapOptimizationInit();
bool exportKeyframeMap(const std::string &directory_path);
bool importKeyframeMap(const std::string &directory_path);

bool registerInitialPoseToKeyframes3D(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &scan_base_3d,
    const Eigen::Vector3d &initial_position_map,
    const Eigen::Quaterniond &initial_orientation_map,
    Eigen::Vector3d &registered_position_map,
    Eigen::Quaterniond &registered_orientation_map,
    std::string *failure_reason = nullptr);

void saveKeyFramesAndFactor(
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr feats_down_body,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr feats_undistort);

void correctPoses();

void publishSamMsg();

void loopClosureThread();

void setLaserCurTime(double lidar_end_time);

void visualizeGlobalMapThread();

#endif
