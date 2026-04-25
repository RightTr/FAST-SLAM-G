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

// GPS Settings
bool useImuHeadingInitialization;
bool useGpsElevation;
float gpsCovThreshold;
float poseCovThreshold;

// CPU Params
int numberOfCores;
double mappingProcessInterval;

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

int ikdtreeSearchNeighborNum;
bool publishMapToOdomTf;

void read_liosam_params() {

    // Topics
    rosparam_get("lio_sam/gpsTopic", gpsTopic, std::string("odometry/gps"));
    rosparam_get("lio_sam/lidarFrame", lidarFrame, std::string("lidar"));
    rosparam_get("lio_sam/baselinkFrame", baselinkFrame, std::string("base_link"));
    rosparam_get("lio_sam/odometryFrame", odometryFrame, std::string("odom"));
    rosparam_get("lio_sam/mapFrame", mapFrame, std::string("map"));
    rosparam_get("lio_sam/highFrequencyBaselinkFrame", highFrequencyBaselinkFrame, std::string(""));
    rosparam_get("lio_sam/publishMapToOdomTf", publishMapToOdomTf, true);
    if (highFrequencyBaselinkFrame.empty()) {
        highFrequencyBaselinkFrame = baselinkFrame + "_hf";
    }

    // GPS settings
    rosparam_get("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
    rosparam_get("lio_sam/useGpsElevation", useGpsElevation, false);
    rosparam_get("lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0f);
    rosparam_get("lio_sam/poseCovThreshold", poseCovThreshold, 25.0f);

    // CPU parameters
    rosparam_get("lio_sam/numberOfCores", numberOfCores, 2);
    rosparam_get("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

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
    rosparam_get("lio_sam/ikdtreeSearchNeighborNum", ikdtreeSearchNeighborNum, 8);
}
