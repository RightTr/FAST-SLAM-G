// Modified from LIO-SAM: MapOptimization.cpp

#include "utility.h"
#include "map_optimization.h"
#include "ros_utils.h"
#include "common_utils.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <unordered_set>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ikd-Tree/ikdtree_public.h"

using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

using PointType = pcl::PointXYZI;

// gtsam
NonlinearFactorGraph gtSAMgraph;
Values initialEstimate;
Values optimizedEstimate;
ISAM2 *isam;
Values isamCurrentEstimate;

Pcl2Publisher pubKeyPoses;
PathPublisher pubPath;

Pcl2Publisher pubLaserCloudGlobal;
Pcl2Publisher pubLaserCloudGlobalDense;
Pcl2Publisher pubLaserCloudLocal;

Pcl2Publisher pubHistoryKeyFrames;
Pcl2Publisher pubIcpKeyFrames;
Pcl2Publisher pubRecentKeyFrame;
Pcl2Publisher pubCloudRegisteredRaw;
MarkerArrayPublisher pubLoopConstraintEdge;

TimeType timeLaserInfoStamp;

pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D; // Store keyframe poses and indexes 
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

double timeLaserInfoCur;

float transformTobeMapped[6];
Eigen::Vector3d translationLidarToIMU;
Eigen::Matrix3d rotationLidarToIMU;

bool isDegenerate = false;

PathMsg globalPath;

std::mutex mtx;
std::mutex mtxLoopInfo;

bool aLoopIsClosed = false;
map<int, int> loopIndexContainer; // from new to old
vector<pair<int, int>> loopIndexQueue;
vector<gtsam::Pose3> loopPoseQueue;
vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;

pcl::VoxelGrid<PointType> downSizeFilterICP;

vector<pcl::PointCloud<PointType>::Ptr> featCloudKeyFrames;
vector<pcl::PointCloud<PointType>::Ptr> denseCloudKeyFrames;

KD_TREE_PUBLIC<PointType>::Ptr ikdtreeHistoryKeyPoses;

KD_TREE_PUBLIC<PointType>::PointVector initPoses3D;

map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;

void publishGlobalMap();
void updatePath(const PointTypePose& pose_in);
pcl::PointCloud<PointType>::Ptr transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloudIn,
    PointTypePose* transformIn);
pcl::PointCloud<PointType>::Ptr transformPointCloud2D(
    pcl::PointCloud<PointType>::Ptr cloudIn,
    PointTypePose* transformIn);
gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);

constexpr const char *kKeyframePosesFile = "pose.pcd";
constexpr const char *kKeyframeMapFolder = "KEYFRAMES";
constexpr const char *kKeyframeCloud2DFolder = "2d";
constexpr const char *kKeyframeCloud3DFolder = "3d";
constexpr float kDynamicHeightBand = 0.6f;
constexpr float kDynamicCellSize = 0.10f;

std::uint64_t denseCellKey(const PointType &point)
{
    const auto cell_x = static_cast<std::int64_t>(std::floor(point.x / kDynamicCellSize));
    const auto cell_y = static_cast<std::int64_t>(std::floor(point.y / kDynamicCellSize));
    return (static_cast<std::uint64_t>(cell_x) << 32) ^
           (static_cast<std::uint64_t>(cell_y) & 0xffffffffULL);
}

void rebuildIsamFromLoadedPoses(const std::vector<PointTypePose> &poses)
{
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    // Imported maps need a finite world anchor before live keyframes are appended.
    noiseModel::Diagonal::shared_ptr priorNoise =
        noiseModel::Diagonal::Variances(
            (Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
    noiseModel::Diagonal::shared_ptr odometryNoise =
        noiseModel::Diagonal::Variances(
            (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

    gtSAMgraph.add(PriorFactor<Pose3>(0, pclPointTogtsamPose3(poses.front()), priorNoise));
    initialEstimate.insert(0, pclPointTogtsamPose3(poses.front()));

    for (std::size_t i = 1; i < poses.size(); ++i)
    {
        const Pose3 pose_from = pclPointTogtsamPose3(poses[i - 1]);
        const Pose3 pose_to = pclPointTogtsamPose3(poses[i]);
        gtSAMgraph.add(BetweenFactor<Pose3>(
            i - 1,
            i,
            pose_from.between(pose_to),
            odometryNoise));
        initialEstimate.insert(i, pose_to);
    }

    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    gtSAMgraph.resize(0);
    initialEstimate.clear();
    isamCurrentEstimate = isam->calculateEstimate();
}

bool exportKeyframeMap(
    const std::string &directory_path)
{
    const std::filesystem::path keyframe_dir =
        std::filesystem::path(directory_path) / kKeyframeMapFolder;
    const std::filesystem::path cloud_2d_dir = keyframe_dir / kKeyframeCloud2DFolder;
    const std::filesystem::path cloud_3d_dir = keyframe_dir / kKeyframeCloud3DFolder;
    if (!create_directory(cloud_2d_dir.string()) ||
        !create_directory(cloud_3d_dir.string()))
        return false;

    const std::size_t keyframe_count = cloudKeyPoses6D->size();
    if (denseCloudKeyFrames.size() < keyframe_count ||
        featCloudKeyFrames.size() < keyframe_count)
        return false;

    pcl::PointCloud<PointTypePose> pose_cloud;
    pose_cloud.points = cloudKeyPoses6D->points;
    pose_cloud.width = pose_cloud.size();
    pose_cloud.height = 1;
    pose_cloud.is_dense = false;
    for (std::size_t i = 0; i < pose_cloud.size(); ++i)
        pose_cloud.points[i].intensity = static_cast<float>(i);

    if (pcl::io::savePCDFileBinary((keyframe_dir / kKeyframePosesFile).string(), pose_cloud) < 0)
        return false;

    for (std::size_t i = 0; i < keyframe_count; ++i)
    {
        if (!denseCloudKeyFrames[i] || !featCloudKeyFrames[i])
            return false;

        std::ostringstream filename_stream;
        filename_stream << std::setw(6) << std::setfill('0') << i << ".pcd";
        if (pcl::io::savePCDFileBinary(
                (cloud_2d_dir / filename_stream.str()).string(),
                *denseCloudKeyFrames[i]) < 0)
            return false;

        if (pcl::io::savePCDFileBinary(
                (cloud_3d_dir / filename_stream.str()).string(),
                *featCloudKeyFrames[i]) < 0)
            return false;
    }

    return true;
}

bool importKeyframeMap(const std::string &directory_path)
{
    const std::filesystem::path keyframe_dir =
        std::filesystem::path(directory_path) / kKeyframeMapFolder;
    const std::filesystem::path pose_path = keyframe_dir / kKeyframePosesFile;
    const std::filesystem::path cloud_2d_dir = keyframe_dir / kKeyframeCloud2DFolder;
    const std::filesystem::path cloud_3d_dir = keyframe_dir / kKeyframeCloud3DFolder;

    pcl::PointCloud<PointTypePose>::Ptr pose_cloud(new pcl::PointCloud<PointTypePose>());
    if (pcl::io::loadPCDFile<PointTypePose>(pose_path.string(), *pose_cloud) < 0 ||
        pose_cloud->empty())
        return false;

    std::vector<PointTypePose> loaded_poses;
    std::vector<pcl::PointCloud<PointType>::Ptr> loaded_dense_clouds;
    std::vector<pcl::PointCloud<PointType>::Ptr> loaded_feat_clouds;
    KD_TREE_PUBLIC<PointType>::PointVector loaded_pose_points;
    loaded_poses.reserve(pose_cloud->size());
    loaded_dense_clouds.reserve(pose_cloud->size());
    loaded_feat_clouds.reserve(pose_cloud->size());
    loaded_pose_points.reserve(pose_cloud->size());

    for (std::size_t i = 0; i < pose_cloud->size(); ++i)
    {
        PointTypePose pose = pose_cloud->points[i];
        pose.intensity = static_cast<float>(i);
        loaded_poses.push_back(pose);

        PointType pose3d;
        pose3d.x = pose.x;
        pose3d.y = pose.y;
        pose3d.z = pose.z;
        pose3d.intensity = static_cast<float>(i);
        loaded_pose_points.push_back(pose3d);

        pcl::PointCloud<PointType>::Ptr loaded_cloud(new pcl::PointCloud<PointType>());
        std::ostringstream filename_stream;
        filename_stream << std::setw(6) << std::setfill('0') << i << ".pcd";
        const std::filesystem::path cloud_2d_path = cloud_2d_dir / filename_stream.str();
        if (pcl::io::loadPCDFile<PointType>(cloud_2d_path.string(), *loaded_cloud) < 0)
            return false;

        pcl::PointCloud<PointType>::Ptr loaded_3d_cloud(new pcl::PointCloud<PointType>());
        const std::filesystem::path cloud_3d_path = cloud_3d_dir / filename_stream.str();
        if (pcl::io::loadPCDFile<PointType>(cloud_3d_path.string(), *loaded_3d_cloud) < 0)
            return false;

        loaded_dense_clouds.push_back(loaded_cloud);
        loaded_feat_clouds.push_back(loaded_3d_cloud);
    }

    rebuildIsamFromLoadedPoses(loaded_poses);

    for (std::size_t i = 0; i < loaded_poses.size(); ++i)
    {
        PointType pose3d;
        pose3d.x = loaded_poses[i].x;
        pose3d.y = loaded_poses[i].y;
        pose3d.z = loaded_poses[i].z;
        pose3d.intensity = static_cast<float>(i);

        cloudKeyPoses3D->push_back(pose3d);
        cloudKeyPoses6D->push_back(loaded_poses[i]);
        copy_cloudKeyPoses3D->push_back(pose3d);
        copy_cloudKeyPoses6D->push_back(loaded_poses[i]);
        featCloudKeyFrames.push_back(loaded_feat_clouds[i]);
        denseCloudKeyFrames.push_back(loaded_dense_clouds[i]);
        initPoses3D.push_back(pose3d);
    }

    ikdtreeHistoryKeyPoses->Build(initPoses3D);

    for (const auto &pose : loaded_poses)
        updatePath(pose);

    return true;
}

Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{ 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

Eigen::Affine3f trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
}

gtsam::Pose3 trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

PointTypePose trans2PointTypePose(float transformIn[])
{
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll  = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw   = transformIn[2];
    return thisPose6D;
}

PointTypePose transformPoseOdomToMap(const PointTypePose &pose)
{
    PointTypePose mapped = pose;
    const Eigen::Quaterniond orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pose.pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(pose.roll, Eigen::Vector3d::UnitX()));
    const Eigen::Vector3d position = transformPositionOdomToMap(Eigen::Vector3d(pose.x, pose.y, pose.z));
    const Eigen::Quaterniond mapped_orientation = transformOrientationOdomToMap(orientation);
    const Eigen::Vector3d mapped_rpy = mapped_orientation.toRotationMatrix().eulerAngles(2, 1, 0);

    mapped.x = position.x();
    mapped.y = position.y();
    mapped.z = position.z();
    mapped.roll = mapped_rpy[2];
    mapped.pitch = mapped_rpy[1];
    mapped.yaw = mapped_rpy[0];
    return mapped;
}

void setLaserCurTime(double lidar_end_time)
{
    timeLaserInfoCur = lidar_end_time;
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

pcl::PointCloud<PointType>::Ptr transformPointCloud2D(
    pcl::PointCloud<PointType>::Ptr cloudIn,
    PointTypePose* transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    const int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    const float cos_yaw = std::cos(transformIn->yaw);
    const float sin_yaw = std::sin(transformIn->yaw);

    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = cos_yaw * pointFrom.x - sin_yaw * pointFrom.y + transformIn->x;
        cloudOut->points[i].y = sin_yaw * pointFrom.x + cos_yaw * pointFrom.y + transformIn->y;
        cloudOut->points[i].z = 0.0f;
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

void allocateMemory()
{
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    ikdtreeHistoryKeyPoses.reset(new KD_TREE_PUBLIC<PointType>());
    featCloudKeyFrames.clear();
    denseCloudKeyFrames.clear();
    initPoses3D.clear();

    for (int i = 0; i < 6; ++i){
        transformTobeMapped[i] = 0;
    }
}

PointType transformFrontendPointToImu(const pcl::PointXYZINormal &pt)
{
    PointType point;
    const Eigen::Vector3d pointBodyLidar(pt.x, pt.y, pt.z);
    const Eigen::Vector3d pointBodyImu(rotationLidarToIMU * pointBodyLidar + translationLidarToIMU);
    point.x = pointBodyImu(0);
    point.y = pointBodyImu(1);
    point.z = pointBodyImu(2);
    point.intensity = pt.intensity;
    return point;
}

void MapOptimizationInit()
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    init_ros_node();
    pubKeyPoses = create_publisher<PointCloud2Msg>("lio_sam/trajectory", 1);
    pubPath = create_publisher<PathMsg>("lio_sam/mapping/path", 1);
    pubLaserCloudGlobal = create_publisher<PointCloud2Msg>("lio_sam/mapping/cloud_global", 1);
    pubLaserCloudGlobalDense = create_publisher<PointCloud2Msg>("lio_sam/mapping/cloud_global_2d", 1);
    pubRecentKeyFrame = create_publisher<PointCloud2Msg>("lio_sam/mapping/cloud_recent_keyframe", 1);
    pubLoopConstraintEdge = create_publisher<MarkerArrayMsg>("lio_sam/loop_closure_constraints", 1);

    downSizeFilterICP.setLeafSize(mappingICPSize, mappingICPSize, mappingICPSize);

    allocateMemory();
}

bool saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
        abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
        return false;
    
    return true;
}  

void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        *nearKeyframes += *transformPointCloud(featCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

bool detectLoopClosureDistance(int *latestID, int *closestID)
{
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;

    // check loop constraint added before
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    if (ikdtreeHistoryKeyPoses->Root_Node == nullptr)
        return false;

    // find the closest history key frame
    KD_TREE_PUBLIC<PointType>::PointVector pointSearchPoses3D;
    std::vector<float> pointSearchSqDisLoop;
    
    ikdtreeHistoryKeyPoses->Nearest_Search(copy_cloudKeyPoses3D->back(), ikdtreeSearchNeighborNum, pointSearchPoses3D, pointSearchSqDisLoop, historyKeyframeSearchRadius);
    for (int i = 0; i < (int)pointSearchPoses3D.size(); ++i)
    {
        int id = pointSearchPoses3D[i].intensity; // index stored in intensity field
        if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
        {
            loopKeyPre = id;
            break;
        }
    }

    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

void performLoopClosure()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    mtx.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    // find keys
    int loopKeyCur;
    int loopKeyPre;
    if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false) return;

    // extract cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    {
        // cloud near latest keyframe 
        loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
        // cloud near previous loop keyframe
        loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
            return;
        // if (pubHistoryKeyFrames.getNumSubscribers() != 0)
        //     publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
    }

    // ICP Settings
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return;

    // publish corrected cloud
    // if (pubIcpKeyFrames.getNumSubscribers() != 0)
    // {
    //     pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
    //     pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
    //     publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
    // }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    // transform from world origin to wrong pose
    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    // transform from world origin to corrected pose
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

    // Add pose constraint
    mtx.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    mtx.unlock();

    // add loop constriant
    loopIndexContainer[loopKeyCur] = loopKeyPre;
}

void addOdomFactor()
{
    if (cloudKeyPoses3D->points.empty())
    {
        // TODO: use IKFoM covariance as prior covariance
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }else{
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
        gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}

void addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

void updatePath(const PointTypePose& pose_in)
{
    const PointTypePose map_pose = transformPoseOdomToMap(pose_in);
    PoseStampedMsg pose_stamped;
    pose_stamped.header.stamp = get_ros_time(map_pose.time);
    pose_stamped.header.frame_id = mapFrame;
    pose_stamped.pose.position.x = map_pose.x;
    pose_stamped.pose.position.y = map_pose.y;
    pose_stamped.pose.position.z = map_pose.z;
    pose_stamped.pose.orientation = quaternion_from_rpy(map_pose.roll, map_pose.pitch, map_pose.yaw);

    globalPath.poses.push_back(pose_stamped);
}

void saveKeyFramesAndFactor(
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr feats_down_body,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr feats_undistort)
{
    if (!cloudKeyPoses3D->points.empty() && saveFrame() == false)
        return;

    // odom factor
    addOdomFactor();

    // loop factor
    addLoopFactor();

    // cout << "****************************************************" << endl;
    // gtSAMgraph.print("GTSAM Graph:\n");

    // update iSAM
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    if (aLoopIsClosed == true)
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    //save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;
    Pose3 latestEstimate;

    isamCurrentEstimate = isam->calculateEstimate();
    latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
    // cout << "****************************************************" << endl;
    // isamCurrentEstimate.print("Current estimate: ");

    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
    cloudKeyPoses3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
    thisPose6D.roll  = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw   = latestEstimate.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);

    updatePath(thisPose6D);

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
    // poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

    // save updated transform
    transformTobeMapped[0] = latestEstimate.rotation().roll();
    transformTobeMapped[1] = latestEstimate.rotation().pitch();
    transformTobeMapped[2] = latestEstimate.rotation().yaw();
    transformTobeMapped[3] = latestEstimate.translation().x();
    transformTobeMapped[4] = latestEstimate.translation().y();
    transformTobeMapped[5] = latestEstimate.translation().z();

    pcl::PointCloud<PointType>::Ptr featCloudKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr denseCloudKeyFrame(new pcl::PointCloud<PointType>());
    featCloudKeyFrame->reserve(feats_down_body->points.size());
    denseCloudKeyFrame->reserve(feats_undistort->points.size());

    for (const auto &pt : feats_down_body->points) {
        featCloudKeyFrame->push_back(transformFrontendPointToImu(pt));
    }

    std::unordered_set<std::uint64_t> dynamic_cells;
    for (const auto &pt : feats_undistort->points) {
        const PointType point = transformFrontendPointToImu(pt);
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            continue;

        if (point.z > scanSliceMaxZ && point.z <= scanSliceMaxZ + kDynamicHeightBand)
            dynamic_cells.insert(denseCellKey(point));
    }

    for (const auto &pt : feats_undistort->points) {
        PointType point = transformFrontendPointToImu(pt);
        if (std::isfinite(point.x) &&
            std::isfinite(point.y) &&
            std::isfinite(point.z) &&
            point.z >= scanSliceMinZ &&
            point.z <= scanSliceMaxZ &&
            dynamic_cells.find(denseCellKey(point)) == dynamic_cells.end()) {
            point.z = 0.0f;
            denseCloudKeyFrame->push_back(point);
        }
    }

    denseCloudKeyFrame = projectCloudToScanPoints(
        *denseCloudKeyFrame,
        [](const PointType &point, pcl::PointXYZI &scan_point) {
            scan_point = point;
            return true;
        });

    featCloudKeyFrames.push_back(featCloudKeyFrame);
    denseCloudKeyFrames.push_back(denseCloudKeyFrame);

    if (ikdtreeHistoryKeyPoses->Root_Node == nullptr) {
        initPoses3D.push_back(thisPose3D);
        ikdtreeHistoryKeyPoses->Build(initPoses3D);
    } else {
        ikdtreeHistoryKeyPoses->Add_Point(thisPose3D);
    }

}

void ReconstructIkdTree()
{
    if (ikdtreeHistoryKeyPoses->Root_Node == nullptr)
        return;
    if (cloudKeyPoses3D->points.empty())
        return;

    ikdtreeHistoryKeyPoses->delete_tree_nodes(&ikdtreeHistoryKeyPoses->Root_Node);

    KD_TREE_PUBLIC<PointType>::PointVector pose_points;
    pose_points.reserve(cloudKeyPoses3D->points.size());
    for (const auto &pose : cloudKeyPoses3D->points)
    {
        pose_points.push_back(pose);
    }

    ikdtreeHistoryKeyPoses->Build(pose_points);
}

void correctPoses()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true)
    {
        // clear path
        globalPath.poses.clear();
        // update key poses
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

            updatePath(cloudKeyPoses6D->points[i]);
        }

        ReconstructIkdTree();
        aLoopIsClosed = false;
    }
}

void publishSamMsg()
{
    if (cloudKeyPoses3D->points.empty())
        return;
    // publish key poses
    pcl::PointCloud<PointType>::Ptr keyPosesMap(new pcl::PointCloud<PointType>());
    keyPosesMap->reserve(cloudKeyPoses3D->points.size());
    for (const auto &pose : cloudKeyPoses3D->points) {
        PointType mapped_pose = pose;
        transformPointOdomToMapInPlace(mapped_pose);
        keyPosesMap->push_back(mapped_pose);
    }
    publishCloud(pubKeyPoses, keyPosesMap, timeLaserInfoStamp, mapFrame);
    if (ros_subscription_count(pubPath) != 0)
    {
        globalPath.header.stamp = timeLaserInfoStamp;
        globalPath.header.frame_id = mapFrame;
        ros_publish(pubPath, globalPath);
    }

    if (ros_subscription_count(pubRecentKeyFrame) != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        *cloudOut += *transformPointCloud(featCloudKeyFrames.back(), &thisPose6D);
        pcl::PointCloud<PointType>::Ptr mappedCloud(new pcl::PointCloud<PointType>(*cloudOut));
        for (auto &point : mappedCloud->points)
            transformPointOdomToMapInPlace(point);
        publishCloud(pubRecentKeyFrame, mappedCloud, timeLaserInfoStamp, mapFrame);
    }
}

void visualizeLoopClosure()
{
    if (loopIndexContainer.empty())
        return;
    
    MarkerArrayMsg markerArray;
    // loop nodes
    MarkerMsg markerNode;
    markerNode.header.frame_id = mapFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = MarkerMsg::ADD;
    markerNode.type = MarkerMsg::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.1; markerNode.scale.y = 0.1; markerNode.scale.z = 0.1; 
    markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
    markerNode.color.a = 1;
    // loop edges
    MarkerMsg markerEdge;
    markerEdge.header.frame_id = mapFrame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = MarkerMsg::ADD;
    markerEdge.type = MarkerMsg::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        PointMsg p;
        PointType point_cur = copy_cloudKeyPoses3D->points[key_cur];
        transformPointOdomToMapInPlace(point_cur);
        p.x = point_cur.x;
        p.y = point_cur.y;
        p.z = point_cur.z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        PointType point_pre = copy_cloudKeyPoses3D->points[key_pre];
        transformPointOdomToMapInPlace(point_pre);
        p.x = point_pre.x;
        p.y = point_pre.y;
        p.z = point_pre.z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    ros_publish(pubLoopConstraintEdge, markerArray);
}

void loopClosureThread()
{
    if (loopClosureEnableFlag == false)
        return;

    ROS_PRINT_INFO("...... Loop Closure Thread Start......");

    RateType rate(loopClosureFrequency);
    while (ros_ok() && !flg_exit)
    {
        rate.sleep();
        performLoopClosure();
        visualizeLoopClosure();
    }
}

void publishGlobalMap() {
    const bool need_sparse_global = ros_subscription_count(pubLaserCloudGlobal) != 0;
    const bool need_dense_global = occupancyMapEnabled || ros_subscription_count(pubLaserCloudGlobalDense) != 0;
    if (!need_sparse_global && !need_dense_global)
        return;

    if (cloudKeyPoses3D->points.empty() || ikdtreeHistoryKeyPoses->Root_Node == nullptr)
        return;

    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapDenseKeyFrames(new pcl::PointCloud<PointType>());

    KD_TREE_PUBLIC<PointType>::PointVector globalMapSearchPoses3D;
    std::vector<float> pointSearchSqDisGlobalMap;
    // search near key frames to visualize
    mtx.lock();
    ikdtreeHistoryKeyPoses->Radius_Search(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, globalMapSearchPoses3D);
    mtx.unlock();

    for (int i = 0; i < (int)globalMapSearchPoses3D.size(); ++i)
        globalMapKeyPoses->push_back(cloudKeyPoses3D->points[globalMapSearchPoses3D[i].intensity]); // index stored in intensity field
    // downsample near selected key frames
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
    for(auto& pt : globalMapKeyPosesDS->points)
    {
        ikdtreeHistoryKeyPoses->Nearest_Search(pt, 1, globalMapSearchPoses3D, pointSearchSqDisGlobalMap);
        if (globalMapSearchPoses3D.empty())
            continue;
        pt.intensity = cloudKeyPoses3D->points[globalMapSearchPoses3D[0].intensity].intensity;
    }

    // extract visualized and downsampled key frames
    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
        if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->points.back()) > globalMapVisualizationSearchRadius)
            continue;
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        if (need_sparse_global) {
            *globalMapKeyFrames += *transformPointCloud(featCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        }
        if (need_dense_global) {
            *globalMapDenseKeyFrames += *transformPointCloud2D(denseCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        }
    }
    if (need_sparse_global) {
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        pcl::PointCloud<PointType>::Ptr mappedGlobalMapKeyFramesDS(
            new pcl::PointCloud<PointType>(*globalMapKeyFramesDS));
        for (auto &point : mappedGlobalMapKeyFramesDS->points)
            transformPointOdomToMapInPlace(point);
        publishCloud(pubLaserCloudGlobal, mappedGlobalMapKeyFramesDS, timeLaserInfoStamp, mapFrame);
    }
    if (need_dense_global) {
        pcl::PointCloud<PointType>::Ptr mappedGlobalMapDenseKeyFrames(
            new pcl::PointCloud<PointType>(*globalMapDenseKeyFrames));
        for (auto &point : mappedGlobalMapDenseKeyFrames->points)
            transformPointOdomToMapInPlace(point);
        PointCloud2Msg denseCloudMsg;
        pcl::toROSMsg(*mappedGlobalMapDenseKeyFrames, denseCloudMsg);
        denseCloudMsg.header.stamp = timeLaserInfoStamp;
        denseCloudMsg.header.frame_id = mapFrame;
        ros_publish(pubLaserCloudGlobalDense, denseCloudMsg);
    }
}

void visualizeGlobalMapThread()
{
    RateType rate(0.2);
    while (ros_ok() && !flg_exit){
        rate.sleep();
        publishGlobalMap();
    }
}
