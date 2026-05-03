#include "utility.h"
#include "map_optimization.h"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include "ikd-Tree/ikdtree_public.h"

using PointType = pcl::PointXYZI;

extern std::mutex mtx;
extern pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
extern pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
extern std::vector<pcl::PointCloud<PointType>::Ptr> featCloudKeyFrames;
extern KD_TREE_PUBLIC<PointType>::Ptr ikdtreeHistoryKeyPoses;

pcl::PointCloud<PointType>::Ptr transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloudIn,
    PointTypePose *transformIn);

namespace
{
constexpr double kCoarseMaxCorr = 2.0;
constexpr double kFineMaxCorr = 0.5;
constexpr int kIcpIterations = 100;
constexpr float kLeafSize = 0.05f;
constexpr double kMaxDeltaXY = 2.0;
constexpr double kMaxDeltaYaw = 0.8;
constexpr int kMaxKeyframes = 15;

bool run_icp(
    const pcl::PointCloud<PointType>::ConstPtr &source_ds,
    const pcl::PointCloud<PointType>::ConstPtr &target_ds,
    const double max_corr,
    const Eigen::Matrix4f &guess,
    Eigen::Matrix4f &final_transform,
    double &fitness_score)
{
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(max_corr);
    icp.setMaximumIterations(kIcpIterations);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-8);
    icp.setRANSACIterations(0);
    icp.setInputSource(source_ds);
    icp.setInputTarget(target_ds);

    pcl::PointCloud<PointType> aligned;
    icp.align(aligned, guess);

    final_transform = icp.getFinalTransformation();
    fitness_score = icp.getFitnessScore(max_corr);
    return icp.hasConverged();
}
}

bool registerInitialPoseToKeyframes3D(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &scan_base_3d,
    const Eigen::Vector3d &initial_position_map,
    const Eigen::Quaterniond &initial_orientation_map,
    Eigen::Vector3d &registered_position_map,
    Eigen::Quaterniond &registered_orientation_map,
    std::string *)
{
    pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>());
    source->reserve(scan_base_3d->size());
    for (const auto &point : scan_base_3d->points)
    {
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z))
            continue;

        PointType source_point;
        source_point.x = point.x;
        source_point.y = point.y;
        source_point.z = point.z;
        source_point.intensity = point.intensity;
        source->push_back(source_point);
    }

    pcl::PointCloud<PointType>::Ptr target(new pcl::PointCloud<PointType>());
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!cloudKeyPoses3D || cloudKeyPoses3D->empty() ||
            !cloudKeyPoses6D || !ikdtreeHistoryKeyPoses ||
            ikdtreeHistoryKeyPoses->Root_Node == nullptr)
            return false;

        PointType query;
        query.x = static_cast<float>(initial_position_map.x());
        query.y = static_cast<float>(initial_position_map.y());
        query.z = static_cast<float>(initial_position_map.z());
        query.intensity = 0.0f;

        KD_TREE_PUBLIC<PointType>::PointVector near_keyposes;
        ikdtreeHistoryKeyPoses->Radius_Search(
            query,
            static_cast<float>(init_reg_search_radius),
            near_keyposes);

        if (static_cast<int>(near_keyposes.size()) > kMaxKeyframes)
        {
            std::sort(
                near_keyposes.begin(),
                near_keyposes.end(),
                [&query](const PointType &lhs, const PointType &rhs) {
                    const float ldx = lhs.x - query.x;
                    const float ldy = lhs.y - query.y;
                    const float ldz = lhs.z - query.z;
                    const float rdx = rhs.x - query.x;
                    const float rdy = rhs.y - query.y;
                    const float rdz = rhs.z - query.z;
                    return ldx * ldx + ldy * ldy + ldz * ldz <
                           rdx * rdx + rdy * rdy + rdz * rdz;
                });
            near_keyposes.resize(kMaxKeyframes);
        }

        for (const auto &near_pose : near_keyposes)
        {
            const int key_index = static_cast<int>(std::lround(near_pose.intensity));
            if (key_index < 0 ||
                key_index >= static_cast<int>(cloudKeyPoses6D->size()) ||
                key_index >= static_cast<int>(featCloudKeyFrames.size()) ||
                !featCloudKeyFrames[key_index] ||
                featCloudKeyFrames[key_index]->empty())
                continue;

            *target += *transformPointCloud(
                featCloudKeyFrames[key_index],
                &cloudKeyPoses6D->points[key_index]);
        }
    }

    pcl::VoxelGrid<PointType> downsample_filter;
    downsample_filter.setLeafSize(kLeafSize, kLeafSize, kLeafSize);

    pcl::PointCloud<PointType>::Ptr source_ds(new pcl::PointCloud<PointType>());
    downsample_filter.setInputCloud(source);
    downsample_filter.filter(*source_ds);

    pcl::PointCloud<PointType>::Ptr target_ds(new pcl::PointCloud<PointType>());
    downsample_filter.setInputCloud(target);
    downsample_filter.filter(*target_ds);

    const Eigen::Vector3d euler =
        initial_orientation_map.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
    const float initial_yaw = static_cast<float>(euler[0]);
    const Eigen::Matrix4f initial_guess =
        pcl::getTransformation(
            static_cast<float>(initial_position_map.x()),
            static_cast<float>(initial_position_map.y()),
            static_cast<float>(initial_position_map.z()),
            0.0f,
            0.0f,
            initial_yaw).matrix();

    Eigen::Matrix4f icp_guess = initial_guess;
    double fitness_score = 0.0;
    if (!run_icp(source_ds, target_ds, kCoarseMaxCorr, initial_guess, icp_guess, fitness_score))
        return false;

    Eigen::Matrix4f final_icp_transform = icp_guess;
    if (!run_icp(source_ds, target_ds, kFineMaxCorr, icp_guess, final_icp_transform, fitness_score))
        return false;

    if (init_reg_fitness_score > 0.0 &&
        fitness_score > init_reg_fitness_score)
        return false;

    const Eigen::Affine3f final_transform(final_icp_transform);
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(final_transform, x, y, z, roll, pitch, yaw);

    const double dx = static_cast<double>(x) - initial_position_map.x();
    const double dy = static_cast<double>(y) - initial_position_map.y();
    const double translation_correction = std::hypot(dx, dy);
    const double yaw_correction = std::atan2(
        std::sin(static_cast<double>(yaw) - static_cast<double>(initial_yaw)),
        std::cos(static_cast<double>(yaw) - static_cast<double>(initial_yaw)));
    if (translation_correction > kMaxDeltaXY ||
        std::abs(yaw_correction) > kMaxDeltaYaw)
        return false;

    registered_position_map = Eigen::Vector3d(x, y, z);
    registered_orientation_map =
        Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).normalized();
    return true;
}
