#ifndef FAST_LIO_SAM_ROS_UTILS_H
#define FAST_LIO_SAM_ROS_UTILS_H

#include <cmath>
#include <cstdarg>
#include <cstdio>

#ifdef USE_ROS1
#include <ros/ros.h>
#elif defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#endif

inline bool ros_ok() {
#ifdef USE_ROS1
    return ros::ok();
#elif defined(USE_ROS2)
    return rclcpp::ok();
#else
    return false;
#endif
}

inline void ROS_PRINT_WARN(const char *fmt, ...) {
    char msg[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);
#ifdef USE_ROS1
    ROS_WARN("%s", msg);
#elif defined(USE_ROS2)
    RCLCPP_WARN(rclcpp::get_logger("fast_lio_sam"), "%s", msg);
#endif
}

inline void ROS_PRINT_ERROR(const char *fmt, ...) {
    char msg[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);
#ifdef USE_ROS1
    ROS_ERROR("%s", msg);
#elif defined(USE_ROS2)
    RCLCPP_ERROR(rclcpp::get_logger("fast_lio_sam"), "%s", msg);
#endif
}

inline void ROS_PRINT_INFO(const char *fmt, ...) {
    char msg[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);
#ifdef USE_ROS1
    ROS_INFO("%s", msg);
#elif defined(USE_ROS2)
    RCLCPP_INFO(rclcpp::get_logger("fast_lio_sam"), "%s", msg);
#endif
}

#ifdef USE_ROS1
inline void spin_once() {
    ros::spinOnce();
}
#elif defined(USE_ROS2)
inline void spin_once(const rclcpp::Node::SharedPtr &node) {
    rclcpp::spin_some(node);
}
#endif

#ifdef USE_ROS1
inline ros::Time get_ros_time(double stamp_sec)
{
    return ros::Time().fromSec(stamp_sec);
}
inline double get_ros_time_sec(const ros::Time &stamp)
{
    return stamp.toSec();
}
#elif defined(USE_ROS2)
inline rclcpp::Time get_ros_time(double stamp_sec)
{
    const int32_t sec = static_cast<int32_t>(std::floor(stamp_sec));
    const uint32_t nanosec = static_cast<uint32_t>((stamp_sec - sec) * 1e9);
    return rclcpp::Time(sec, nanosec);
}
inline double get_ros_time_sec(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}
#endif

// Generic publish wrapper for both ROS1 and ROS2
#ifdef USE_ROS1
template<typename PubType, typename MsgType>
inline void ros_publish(PubType &pub, const MsgType &msg) {
    pub.publish(msg);
}
#elif defined(USE_ROS2)
template<typename PubType, typename MsgType>
inline void ros_publish(const PubType &pub, const MsgType &msg) {
    pub->publish(msg);
}
#endif

#endif
