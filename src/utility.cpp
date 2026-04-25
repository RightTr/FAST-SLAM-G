#include "utility.h"
#include "ros_utils.h"

using namespace std;

//Topics
string gpsTopic;
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
float surroundingKeyframeDensity;
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

bool scanSliceEnable;
float scanSliceMinZ;
float scanSliceMaxZ;
float scanAngleMin;
float scanAngleMax;
float scanAngleIncrement;
float scanTime;
float scanRangeMin;
float scanRangeMax;

int ikdtreeSearchNeighborNum;
bool publishMapToOdomTf;

void read_liosam_params() {

    // Topics
    rosparam_get("lio_sam/gpsTopic", gpsTopic, std::string("odometry/gps"));
    rosparam_get("lio_sam/lidarFrame", lidarFrame, std::string("lidar"));
    rosparam_get("lio_sam/baselinkFrame", baselinkFrame, std::string("base_link"));
    rosparam_get("lio_sam/odometryFrame", odometryFrame, std::string("odom"));
    rosparam_get("lio_sam/mapFrame", mapFrame, std::string("map"));
    rosparam_get("lio_sam/highFrequencyBaselinkFrame", highFrequencyBaselinkFrame, std::string("base_link_hf"));
    rosparam_get("lio_sam/publishMapToOdomTf", publishMapToOdomTf, true);

    // CPU parameters
    rosparam_get("lio_sam/numberOfCores", numberOfCores, 2);

    // Keyframe Strategy
    rosparam_get("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0f);
    rosparam_get("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2f);
    rosparam_get("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0f);
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
    rosparam_get("pointcloud_to_laserscan/slice_enable", scanSliceEnable, true);
    rosparam_get("pointcloud_to_laserscan/min_height", scanSliceMinZ, -1.5f);
    rosparam_get("pointcloud_to_laserscan/max_height", scanSliceMaxZ, 0.8f);
    rosparam_get("pointcloud_to_laserscan/angle_min", scanAngleMin, -3.14159f);
    rosparam_get("pointcloud_to_laserscan/angle_max", scanAngleMax, 3.14159f);
    rosparam_get("pointcloud_to_laserscan/angle_increment", scanAngleIncrement, 0.00436f);
    rosparam_get("pointcloud_to_laserscan/scan_time", scanTime, 0.1f);
    rosparam_get("pointcloud_to_laserscan/range_min", scanRangeMin, 0.1f);
    rosparam_get("pointcloud_to_laserscan/range_max", scanRangeMax, 100.0f);
    rosparam_get("lio_sam/ikdtreeSearchNeighborNum", ikdtreeSearchNeighborNum, 8);
}
