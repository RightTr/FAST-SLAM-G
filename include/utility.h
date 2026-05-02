#ifndef UTILITY_H
#define UTILITY_H

#include <string>
#include <deque>
#include <vector>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ros_utils.h"

//Topics
extern std::string lidarFrame;
extern std::string IMUlinkFrame;
extern std::string baseLinkFrame;
extern std::string odometryFrame;
extern std::string mapFrame;
extern std::string highFrequencyIMUlinkFrame;
extern Eigen::Vector3d baseLinkToLidarTranslation;
extern Eigen::Quaterniond baseLinkToLidarRotation;

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
extern float scanAngleMin;
extern float scanAngleMax;
extern float scanRangeMax;
inline constexpr float scanAngleIncrement = 0.00436f;
inline constexpr float scanTime = 0.1f;
inline constexpr float scanRangeMin = 0.1f;

extern double init_reg_search_radius;
extern double init_reg_fitness_score;

extern int ikdtreeSearchNeighborNum;
extern bool occupancyMapEnabled;
extern bool mapFrameOriginInitialized;
extern Eigen::Quaterniond mapFrameRotationFromOdom;
extern Eigen::Vector3d mapFrameTranslationFromOdom;

extern std::atomic<bool> flg_exit;

void read_frame_params();
void read_pcl2scan_params();
void read_reloc_params();
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

template<typename CloudT, typename ProjectPointFn>
std::vector<float> projectCloudToScanRanges(const CloudT &cloud, ProjectPointFn project_point)
{
    std::vector<float> ranges;
    if (cloud.empty() || scanAngleMax <= scanAngleMin || scanAngleIncrement <= 0.0f)
        return ranges;

    const int beam_count = std::max(1, static_cast<int>(
        std::ceil((scanAngleMax - scanAngleMin) / scanAngleIncrement)));
    ranges.assign(beam_count, std::numeric_limits<float>::infinity());

    for (const auto &point : cloud.points)
    {
        pcl::PointXYZI scan_point;
        if (!project_point(point, scan_point))
            continue;

        if (!std::isfinite(scan_point.x) || !std::isfinite(scan_point.y))
            continue;

        const float range = std::hypot(scan_point.x, scan_point.y);
        if (range < scanRangeMin || range > scanRangeMax)
            continue;

        const float angle = std::atan2(scan_point.y, scan_point.x);
        if (angle < scanAngleMin || angle > scanAngleMax)
            continue;

        const int index = static_cast<int>((angle - scanAngleMin) / scanAngleIncrement);
        if (index < 0 || index >= beam_count)
            continue;

        ranges[index] = std::min(ranges[index], range);
    }

    return ranges;
}

template<typename CloudT, typename ProjectPointFn>
pcl::PointCloud<pcl::PointXYZI>::Ptr projectCloudToScanPoints(const CloudT &cloud, ProjectPointFn project_point)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>());
    if (cloud.empty() || scanAngleMax <= scanAngleMin || scanAngleIncrement <= 0.0f)
        return points;

    const int beam_count = std::max(1, static_cast<int>(
        std::ceil((scanAngleMax - scanAngleMin) / scanAngleIncrement)));
    std::vector<float> best_ranges(beam_count, std::numeric_limits<float>::infinity());
    std::vector<float> best_x(beam_count, 0.0f);
    std::vector<float> best_y(beam_count, 0.0f);
    std::vector<float> best_intensity(beam_count, 0.0f);

    for (const auto &point : cloud.points)
    {
        pcl::PointXYZI scan_point;
        if (!project_point(point, scan_point))
            continue;

        if (!std::isfinite(scan_point.x) || !std::isfinite(scan_point.y))
            continue;

        const float range = std::hypot(scan_point.x, scan_point.y);
        if (range < scanRangeMin || range > scanRangeMax)
            continue;

        const float angle = std::atan2(scan_point.y, scan_point.x);
        if (angle < scanAngleMin || angle > scanAngleMax)
            continue;

        const int index = static_cast<int>((angle - scanAngleMin) / scanAngleIncrement);
        if (index < 0 || index >= beam_count)
            continue;

        if (range < best_ranges[index])
        {
            best_ranges[index] = range;
            best_x[index] = scan_point.x;
            best_y[index] = scan_point.y;
            best_intensity[index] = scan_point.intensity;
        }
    }

    points->reserve(beam_count);
    for (int i = 0; i < beam_count; ++i)
    {
        if (!std::isfinite(best_ranges[i]))
            continue;

        pcl::PointXYZI scan_point;
        scan_point.x = best_x[i];
        scan_point.y = best_y[i];
        scan_point.z = 0.0f;
        scan_point.intensity = best_intensity[i];
        points->push_back(scan_point);
    }

    return points;
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
