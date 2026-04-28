#ifndef UTILITY_H
#define UTILITY_H

#include <string>
#include <deque>
#include <vector>
#include <atomic>

#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include "ros_utils.h"

//Topics
extern std::string lidarFrame;
extern std::string baselinkFrame;
extern std::string odometryFrame;
extern std::string mapFrame;
extern std::string highFrequencyBaselinkFrame;

// CPU Params
extern int numberOfCores;

// Surrounding map
extern float surroundingkeyframeAddingDistThreshold; 
extern float surroundingkeyframeAddingAngleThreshold; 
extern float surroundingKeyframeSearchRadius;

// Loop closure
extern bool  loopClosureEnableFlag;
extern float loopClosureFrequency;
extern int   surroundingKeyframeSize;
extern float historyKeyframeSearchRadius;
extern float historyKeyframeSearchTimeDiff;
extern int   historyKeyframeSearchNum;
extern float historyKeyframeFitnessScore;

// global map visualization radius
extern float globalMapVisualizationSearchRadius;
extern float globalMapVisualizationPoseDensity;
extern float globalMapVisualizationLeafSize;

extern float mappingICPSize;

extern float scanSliceMinZ;
extern float scanSliceMaxZ;
inline constexpr float scanAngleMin = -3.14159f;
inline constexpr float scanAngleMax = 3.14159f;
inline constexpr float scanAngleIncrement = 0.00436f;
inline constexpr float scanTime = 0.1f;
inline constexpr float scanRangeMin = 0.1f;
inline constexpr float scanRangeMax = 100.0f;

extern int ikdtreeSearchNeighborNum;
extern bool occupancyMapEnabled;
extern bool mapFrameOriginInitialized;
extern Eigen::Quaterniond mapFrameRotationFromOdom;
extern Eigen::Vector3d mapFrameTranslationFromOdom;

extern std::atomic<bool> flg_exit;

void read_frame_params();
void read_pcl2scan_params();
void read_liosam_params();
void setMapFrameOriginFromPose(const Eigen::Vector3d &origin_position,
                               const Eigen::Quaterniond &origin_orientation);
inline Eigen::Vector3d transformPositionOdomToMap(const Eigen::Vector3d &position)
{
    return mapFrameRotationFromOdom * position + mapFrameTranslationFromOdom;
}

inline Eigen::Quaterniond transformOrientationOdomToMap(const Eigen::Quaterniond &orientation)
{
    return (mapFrameRotationFromOdom * orientation).normalized();
}

template<typename PointT>
inline void transformPointOdomToMapInPlace(PointT &point)
{
    const Eigen::Vector3d position =
        transformPositionOdomToMap(Eigen::Vector3d(point.x, point.y, point.z));
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
}

template<typename T>
void publishCloud(Pcl2Publisher &thisPub, const T &thisCloud, TimeType thisStamp, const std::string &thisFrame)
{
    PointCloud2Msg tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (ros_subscription_count(thisPub) != 0)
        ros_publish(thisPub, tempCloud);
}

#endif
