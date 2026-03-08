#ifndef UTILITY_H
#define UTILITY_H

#include <string>
#include <deque>
#include <vector>

//Topics
extern std::string gpsTopic;

// GPS Settings
extern bool useImuHeadingInitialization;
extern bool useGpsElevation;
extern float gpsCovThreshold;
extern float poseCovThreshold;

// CPU Params
extern int numberOfCores;
extern double mappingProcessInterval;

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

extern int ikdtreeSearchNeighborNum;

#ifdef USE_ROS1
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

template<typename T>
void publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
}

void read_liosam_params(ros::NodeHandle& nh);

#elif defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>

#endif

#endif