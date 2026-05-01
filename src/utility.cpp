#include "utility.h"
#include "ros_utils.h"
#include "common_lib.h"

using namespace std;

namespace
{
Eigen::Quaterniond yawOnlyQuaternion(const Eigen::Quaterniond &orientation)
{
    const Eigen::Vector3d euler = orientation.toRotationMatrix().eulerAngles(2, 1, 0);
    return Eigen::Quaterniond(Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ())).normalized();
}
} // namespace

// Topics
string lidarFrame;
string baselinkFrame;
string odometryFrame;
string mapFrame;
string highFrequencyBaselinkFrame;

// CPU Params
int numberOfCores;

// Surrounding map
float surroundingkeyframeAddingDistThreshold; 
float surroundingkeyframeAddingAngleThreshold; 
float surroundingKeyframeSearchRadius;

// Loop closure
bool  loopClosureEnableFlag;
float loopClosureFrequency;
int   surroundingKeyframeSize;
float historyKeyframeSearchRadius;
float historyKeyframeSearchTimeDiff;
int   historyKeyframeSearchNum;
float historyKeyframeFitnessScore;

// global map visualization radius
float globalMapVisualizationSearchRadius;
float globalMapVisualizationPoseDensity;
float globalMapVisualizationLeafSize;

float mappingICPSize;

float scanSliceMinZ = -1.5f;
float scanSliceMaxZ = 0.8f;
float scan_angle_min_deg = -180.0f;
float scan_angle_max_deg = 180.0f;
float scanAngleMin = -3.14159f;
float scanAngleMax = 3.14159f;

double init_reg_search_radius = 8.0;
double init_reg_coarse_max_correspondence_distance = 2.0;
double init_reg_max_correspondence_distance = 0.2;
double init_reg_fitness_score = 0.01;
double init_reg_leaf_size = 0.05;
double init_reg_max_translation_correction = 0.8;
double init_reg_max_yaw_correction = 0.35;
double init_reg_min_inlier_ratio = 0.6;
int init_reg_max_keyframes = 20;
int init_reg_min_source_points = 50;
int init_reg_min_target_points = 200;

int ikdtreeSearchNeighborNum;
bool occupancyMapEnabled = true;
bool mapFrameOriginInitialized = false;
Eigen::Quaterniond mapFrameRotationFromOdom = Eigen::Quaterniond::Identity();
Eigen::Vector3d mapFrameTranslationFromOdom = Eigen::Vector3d::Zero();

void read_liosam_params() {
    // CPU parameters
    rosparam_get("lio_sam/numberOfCores", numberOfCores, 2);

    // Keyframe Strategy
    rosparam_get("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0f);
    rosparam_get("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2f);
    rosparam_get("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0f);

    // Loop closure parameters
    rosparam_get("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
    rosparam_get("lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0f);
    rosparam_get("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
    rosparam_get("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0f);
    rosparam_get("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0f);
    rosparam_get("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    rosparam_get("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3f);

    // Global pointcloud visualization
    rosparam_get("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3f);
    rosparam_get("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0f);
    rosparam_get("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0f);

    rosparam_get("lio_sam/mappingICPSize", mappingICPSize, 0.2f);
    rosparam_get("lio_sam/ikdtreeSearchNeighborNum", ikdtreeSearchNeighborNum, 8);
}

void read_frame_params() {
    rosparam_get("common/lidarFrame", lidarFrame, std::string("lidar"));
    rosparam_get("common/baselinkFrame", baselinkFrame, std::string("base_link"));
    rosparam_get("common/odometryFrame", odometryFrame, std::string("odom"));
    rosparam_get("common/mapFrame", mapFrame, std::string("map"));
    rosparam_get("common/highFrequencyBaselinkFrame", highFrequencyBaselinkFrame, std::string("base_link_hf"));
}

void read_pcl2scan_params() {
    rosparam_get("pointcloud_to_laserscan/min_height", scanSliceMinZ, scanSliceMinZ);
    rosparam_get("pointcloud_to_laserscan/max_height", scanSliceMaxZ, scanSliceMaxZ);
    rosparam_get("pointcloud_to_laserscan/angle_min_deg", scan_angle_min_deg, scan_angle_min_deg);
    rosparam_get("pointcloud_to_laserscan/angle_max_deg", scan_angle_max_deg, scan_angle_max_deg);
    scanAngleMin = deg2rad(scan_angle_min_deg);
    scanAngleMax = deg2rad(scan_angle_max_deg);
}

void read_reloc_params() {
    rosparam_get("reloc/init_reg_search_radius", init_reg_search_radius, 8.0);
    rosparam_get("reloc/init_reg_coarse_max_correspondence_distance", init_reg_coarse_max_correspondence_distance, 2.0);
    rosparam_get("reloc/init_reg_max_correspondence_distance", init_reg_max_correspondence_distance, 0.2);
    rosparam_get("reloc/init_reg_fitness_score", init_reg_fitness_score, 0.01);
    rosparam_get("reloc/init_reg_leaf_size", init_reg_leaf_size, 0.05);
    rosparam_get("reloc/init_reg_max_translation_correction", init_reg_max_translation_correction, 0.8);
    rosparam_get("reloc/init_reg_max_yaw_correction", init_reg_max_yaw_correction, 0.35);
    rosparam_get("reloc/init_reg_min_inlier_ratio", init_reg_min_inlier_ratio, 0.6);
    rosparam_get("reloc/init_reg_max_keyframes", init_reg_max_keyframes, 20);
    rosparam_get("reloc/init_reg_min_source_points", init_reg_min_source_points, 50);
    rosparam_get("reloc/init_reg_min_target_points", init_reg_min_target_points, 200);
}

void setMapFrameOriginFromPose(const Eigen::Vector3d &origin_position,
                               const Eigen::Quaterniond &origin_orientation)
{
    const Eigen::Quaterniond origin_yaw = yawOnlyQuaternion(origin_orientation.normalized());
    mapFrameRotationFromOdom = origin_yaw.conjugate().normalized();
    mapFrameTranslationFromOdom = -(mapFrameRotationFromOdom * origin_position);
    mapFrameOriginInitialized = true;
}
