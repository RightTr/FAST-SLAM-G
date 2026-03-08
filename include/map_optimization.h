#ifndef MAP_OPTIMIZATION_H
#define MAP_OPTIMIZATION_H

extern float transformTobeMapped[6];

void MapOptimizationInit();

void saveKeyFramesAndFactor(pcl::PointCloud<pcl::PointXYZINormal>::Ptr feats_undistort);

void correctPoses();

void publishSamMsg();

void loopClosureThread();

void setLaserCurTime(double lidar_end_time);

#endif