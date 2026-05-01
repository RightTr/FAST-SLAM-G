#include "utility.h"
#include "map_optimization.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
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

bool registerInitialPoseToKeyframes3D(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &scan_base_3d,
    const Eigen::Vector3d &initial_position_map,
    const Eigen::Quaterniond &initial_orientation_map,
    Eigen::Vector3d &registered_position_map,
    Eigen::Quaterniond &registered_orientation_map,
    std::string *failure_reason)
{
    if (failure_reason)
        failure_reason->clear();

    const auto set_failure = [failure_reason](const std::string &reason) {
        if (failure_reason)
            *failure_reason = reason;
    };

    if (!scan_base_3d)
    {
        set_failure("source cloud is null");
        return false;
    }

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

    int source_points = static_cast<int>(source->size());
    if (source_points < init_reg_min_source_points)
    {
        std::ostringstream reason;
        reason << "source scan has " << source_points
               << " valid 3D points, need >= " << init_reg_min_source_points
               << " (check current LiDAR scan)";
        set_failure(reason.str());
        return false;
    }

    pcl::PointCloud<PointType>::Ptr target(new pcl::PointCloud<PointType>());
    int candidate_keypose_count = 0;
    int target_keyframe_count = 0;
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!cloudKeyPoses3D || cloudKeyPoses3D->empty() ||
            !cloudKeyPoses6D || !ikdtreeHistoryKeyPoses ||
            ikdtreeHistoryKeyPoses->Root_Node == nullptr)
        {
            std::ostringstream reason;
            reason << "keyframe map is empty or not loaded"
                   << " (key poses=" << (cloudKeyPoses3D ? cloudKeyPoses3D->size() : 0)
                   << ", kd-tree=" << (ikdtreeHistoryKeyPoses && ikdtreeHistoryKeyPoses->Root_Node ? "ready" : "empty")
                   << ")";
            set_failure(reason.str());
            return false;
        }

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
        candidate_keypose_count = static_cast<int>(near_keyposes.size());
        if (near_keyposes.empty())
        {
            std::ostringstream reason;
            reason << std::fixed << std::setprecision(3)
                   << "no keyframes within init_reg_search_radius="
                   << init_reg_search_radius << "m of initial pose (x="
                   << initial_position_map.x() << ", y=" << initial_position_map.y()
                   << ", z=" << initial_position_map.z() << ")";
            set_failure(reason.str());
            return false;
        }

        if (init_reg_max_keyframes > 0 &&
            static_cast<int>(near_keyposes.size()) > init_reg_max_keyframes)
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
            near_keyposes.resize(init_reg_max_keyframes);
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
            ++target_keyframe_count;
        }
    }

    int target_points = static_cast<int>(target->size());
    if (target_points < init_reg_min_target_points)
    {
        std::ostringstream reason;
        reason << "target map has " << target_points
               << " valid 3D points from " << target_keyframe_count
               << "/" << candidate_keypose_count
               << " nearby keyframes, need >= " << init_reg_min_target_points;
        set_failure(reason.str());
        return false;
    }

    const float leaf_size = std::max(0.01f, static_cast<float>(init_reg_leaf_size));
    pcl::VoxelGrid<PointType> downsample_filter;
    downsample_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

    pcl::PointCloud<PointType>::Ptr source_ds(new pcl::PointCloud<PointType>());
    downsample_filter.setInputCloud(source);
    downsample_filter.filter(*source_ds);

    pcl::PointCloud<PointType>::Ptr target_ds(new pcl::PointCloud<PointType>());
    downsample_filter.setInputCloud(target);
    downsample_filter.filter(*target_ds);

    source_points = static_cast<int>(source_ds->size());
    target_points = static_cast<int>(target_ds->size());
    if (source_points < init_reg_min_source_points ||
        target_points < init_reg_min_target_points)
    {
        std::ostringstream reason;
        reason << std::fixed << std::setprecision(3)
               << "downsampled points too few: source=" << source_points
               << " need >= " << init_reg_min_source_points
               << ", target=" << target_points
               << " need >= " << init_reg_min_target_points
               << ", leaf_size=" << leaf_size;
        set_failure(reason.str());
        return false;
    }

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

    auto run_icp = [&source_ds, &target_ds](
        const double max_correspondence_distance,
        const int max_iterations,
        const Eigen::Matrix4f &guess,
        Eigen::Matrix4f &final_transform,
        bool &converged,
        double &fitness_score) {
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(max_correspondence_distance);
        icp.setMaximumIterations(max_iterations);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-8);
        icp.setRANSACIterations(0);
        icp.setInputSource(source_ds);
        icp.setInputTarget(target_ds);

        pcl::PointCloud<PointType> aligned;
        icp.align(aligned, guess);

        converged = icp.hasConverged();
        fitness_score = icp.getFitnessScore();
        final_transform = icp.getFinalTransformation();
    };

    const double fine_max_corr = init_reg_max_correspondence_distance;
    const double coarse_max_corr = std::max(
        init_reg_coarse_max_correspondence_distance,
        init_reg_max_correspondence_distance);
    Eigen::Matrix4f icp_guess = initial_guess;
    bool coarse_converged = true;
    double coarse_fitness_score = 0.0;
    if (coarse_max_corr > fine_max_corr + 1e-6)
    {
        run_icp(
            coarse_max_corr,
            80,
            initial_guess,
            icp_guess,
            coarse_converged,
            coarse_fitness_score);
        if (!coarse_converged)
        {
            std::ostringstream reason;
            reason << std::fixed << std::setprecision(4)
                   << "coarse ICP rejected: converged=false"
                   << ", fitness=" << coarse_fitness_score
                   << ", coarse_max_corr=" << coarse_max_corr
                   << ", source=" << source_points
                   << ", target=" << target_points
                   << ", nearby_keyframes=" << target_keyframe_count
                   << "/" << candidate_keypose_count;
            set_failure(reason.str());
            return false;
        }
    }

    Eigen::Matrix4f final_icp_transform = icp_guess;
    bool converged = false;
    double fitness_score = 0.0;
    run_icp(
        fine_max_corr,
        100,
        icp_guess,
        final_icp_transform,
        converged,
        fitness_score);

    pcl::PointCloud<PointType> source_aligned;
    pcl::transformPointCloud(*source_ds, source_aligned, final_icp_transform);
    pcl::KdTreeFLANN<PointType> target_tree;
    target_tree.setInputCloud(target_ds);
    const double fine_max_corr_sq = fine_max_corr * fine_max_corr;
    double inlier_fitness_score = std::numeric_limits<double>::max();
    double inlier_rmse = std::numeric_limits<double>::max();
    double inlier_ratio = 0.0;
    int inlier_count = 0;
    double inlier_sq_dist_sum = 0.0;
    pcl::Indices nn_indices(1);
    std::vector<float> nn_dists(1);
    for (const auto &point : source_aligned.points)
    {
        if (target_tree.nearestKSearch(point, 1, nn_indices, nn_dists) <= 0)
            continue;

        if (static_cast<double>(nn_dists[0]) > fine_max_corr_sq)
            continue;

        inlier_sq_dist_sum += nn_dists[0];
        ++inlier_count;
    }
    if (!source_aligned.empty())
        inlier_ratio = static_cast<double>(inlier_count) / static_cast<double>(source_aligned.size());
    if (inlier_count > 0)
    {
        inlier_fitness_score = inlier_sq_dist_sum / static_cast<double>(inlier_count);
        inlier_rmse = std::sqrt(inlier_fitness_score);
    }

    if (!converged ||
        inlier_count == 0 ||
        inlier_ratio < init_reg_min_inlier_ratio ||
        (init_reg_fitness_score > 0.0 &&
         inlier_fitness_score > init_reg_fitness_score))
    {
        std::ostringstream reason;
        reason << std::fixed << std::setprecision(4)
               << "ICP rejected: converged=" << (converged ? "true" : "false")
               << ", all_fitness=" << fitness_score
               << ", inlier_fitness=" << inlier_fitness_score
               << ", inlier_rmse=" << inlier_rmse
               << ", inlier_ratio=" << inlier_ratio
               << ", max_fitness=" << init_reg_fitness_score
               << ", min_inlier_ratio=" << init_reg_min_inlier_ratio
               << ", fine_max_corr=" << fine_max_corr
               << ", coarse_max_corr=" << coarse_max_corr
               << ", coarse_fitness=" << coarse_fitness_score
               << ", source=" << source_points
               << ", target=" << target_points
               << ", nearby_keyframes=" << target_keyframe_count
               << "/" << candidate_keypose_count;
        set_failure(reason.str());
        return false;
    }

    const Eigen::Affine3f final_transform(final_icp_transform);
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(final_transform, x, y, z, roll, pitch, yaw);

    const double dx = static_cast<double>(x) - initial_position_map.x();
    const double dy = static_cast<double>(y) - initial_position_map.y();
    const double translation_correction = std::hypot(dx, dy);
    const double yaw_correction = std::atan2(
        std::sin(static_cast<double>(yaw) - static_cast<double>(initial_yaw)),
        std::cos(static_cast<double>(yaw) - static_cast<double>(initial_yaw)));
    if ((init_reg_max_translation_correction > 0.0 &&
         translation_correction > init_reg_max_translation_correction) ||
        (init_reg_max_yaw_correction > 0.0 &&
         std::abs(yaw_correction) > init_reg_max_yaw_correction))
    {
        std::ostringstream reason;
        reason << std::fixed << std::setprecision(4)
               << "ICP rejected: final pose moved too far from RViz initial pose"
               << ", translation_correction=" << translation_correction
               << "m max=" << init_reg_max_translation_correction
               << "m, yaw_correction=" << yaw_correction
               << "rad max=" << init_reg_max_yaw_correction
               << "rad, initial=(" << initial_position_map.x()
               << ", " << initial_position_map.y()
               << ", yaw=" << initial_yaw
               << "), final=(" << x
               << ", " << y
               << ", yaw=" << yaw
               << "), fitness=" << fitness_score;
        set_failure(reason.str());
        return false;
    }

    registered_position_map = Eigen::Vector3d(x, y, z);
    registered_orientation_map =
        Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).normalized();
    return true;
}
