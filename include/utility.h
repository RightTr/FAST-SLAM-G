#ifndef UTILITY_H
#define UTILITY_H

#include <string>
#include <deque>
#include <vector>
#include <atomic>

#include <pcl_conversions/pcl_conversions.h>
#include "ros_utils.h"

//Topics
extern std::string gpsTopic;
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
extern float surroundingKeyframeDensity;
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

extern bool denseKeyframeSliceEnable;
extern float denseKeyframeSliceMinZ;
extern float denseKeyframeSliceMaxZ;
extern bool denseKeyframeSliceFlatten;
extern bool denseKeyframeSliceUseForRecentCloud;
extern bool denseKeyframeSliceUseForGlobalMap;

extern int ikdtreeSearchNeighborNum;
extern bool publishMapToOdomTf;

extern std::atomic<bool> flg_exit;

void read_liosam_params();

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
