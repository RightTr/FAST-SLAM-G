#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ros_utils.h"

using PointType = pcl::PointXYZI;

bool gridmap_enabled = true;
bool fill_free_space = true;
bool accumulate_map = false;
int min_points_per_cell = 1;
double resolution = 0.1;
double padding = 5.0;
double max_size = 0.0;
std::string input_topic = "lio_sam/mapping/cloud_global_2d";
std::string output_topic = "/map";
std::string map_frame = "map";
bool use_current_time = true;

OccupancyGridPublisher pubOccupancyGrid;
std::unordered_set<std::int64_t> accumulated_occupied_cells;

inline std::int64_t makeCellKey(const int cell_x, const int cell_y)
{
    return (static_cast<std::int64_t>(cell_x) << 32) ^
           static_cast<std::uint32_t>(cell_y);
}

inline int keyToCellX(const std::int64_t key)
{
    return static_cast<int>(key >> 32);
}

inline int keyToCellY(const std::int64_t key)
{
    return static_cast<int>(static_cast<std::uint32_t>(key));
}

inline TimeType get_publish_stamp(const Pcl2MsgConstPtr& msg)
{
    if (!use_current_time) {
#ifdef USE_ROS1
        return msg->header.stamp;
#elif defined(USE_ROS2)
        return TimeType(msg->header.stamp);
#endif
    }

#ifdef USE_ROS1
    return get_ros_now();
#elif defined(USE_ROS2)
    return get_ros_now(get_ros_node());
#endif
}

void publishOccupancyFromHits(const TimeType &stamp, const std::vector<std::pair<double, double>> &hits)
{
    std::unordered_map<std::int64_t, int> current_hit_count;
    current_hit_count.reserve(hits.size());
    for (const auto &hit : hits) {
        const int cell_x = static_cast<int>(std::floor(hit.first / resolution));
        const int cell_y = static_cast<int>(std::floor(hit.second / resolution));
        current_hit_count[makeCellKey(cell_x, cell_y)] += 1;
    }

    std::unordered_set<std::int64_t> current_occupied_cells;
    current_occupied_cells.reserve(current_hit_count.size());
    for (const auto &cell_count : current_hit_count) {
        if (cell_count.second >= min_points_per_cell) {
            current_occupied_cells.insert(cell_count.first);
        }
    }

    const std::unordered_set<std::int64_t> *occupied_cells = &current_occupied_cells;
    if (accumulate_map) {
        accumulated_occupied_cells.insert(current_occupied_cells.begin(), current_occupied_cells.end());
        occupied_cells = &accumulated_occupied_cells;
    }

    if (occupied_cells->empty()) {
        return;
    }

    int min_cell_x = std::numeric_limits<int>::max();
    int min_cell_y = std::numeric_limits<int>::max();
    int max_cell_x = std::numeric_limits<int>::lowest();
    int max_cell_y = std::numeric_limits<int>::lowest();

    for (const auto key : *occupied_cells) {
        const int cell_x = keyToCellX(key);
        const int cell_y = keyToCellY(key);
        min_cell_x = std::min(min_cell_x, cell_x);
        min_cell_y = std::min(min_cell_y, cell_y);
        max_cell_x = std::max(max_cell_x, cell_x);
        max_cell_y = std::max(max_cell_y, cell_y);
    }

    const int padding_cells = static_cast<int>(std::ceil(padding / resolution));
    min_cell_x -= padding_cells;
    min_cell_y -= padding_cells;
    max_cell_x += padding_cells;
    max_cell_y += padding_cells;

    if (max_size > 0.0) {
        const int center_cell_x = (min_cell_x + max_cell_x) / 2;
        const int center_cell_y = (min_cell_y + max_cell_y) / 2;
        const int half_size_cells = static_cast<int>(std::ceil(0.5 * max_size / resolution));
        min_cell_x = center_cell_x - half_size_cells;
        max_cell_x = center_cell_x + half_size_cells;
        min_cell_y = center_cell_y - half_size_cells;
        max_cell_y = center_cell_y + half_size_cells;
    }

    const auto width = static_cast<std::uint32_t>(
        std::max(1, max_cell_x - min_cell_x + 1));
    const auto height = static_cast<std::uint32_t>(
        std::max(1, max_cell_y - min_cell_y + 1));
    const double origin_x = static_cast<double>(min_cell_x) * resolution;
    const double origin_y = static_cast<double>(min_cell_y) * resolution;

    OccupancyGridMsg grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = map_frame;
    grid.info.map_load_time = stamp;
    grid.info.resolution = static_cast<float>(resolution);
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(width * height, fill_free_space ? 0 : -1);

    for (const auto key : *occupied_cells) {
        const int cell_x = keyToCellX(key) - min_cell_x;
        const int cell_y = keyToCellY(key) - min_cell_y;
        if (cell_x < 0 || cell_y < 0 ||
            cell_x >= static_cast<int>(width) || cell_y >= static_cast<int>(height)) {
            continue;
        }

        const std::size_t index =
            static_cast<std::size_t>(cell_y) * width + static_cast<std::size_t>(cell_x);
        grid.data[index] = 100;
    }

    ros_publish(pubOccupancyGrid, grid);
}

void cloudCallback(const Pcl2MsgConstPtr& msg)
{
    pcl::PointCloud<PointType> cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::vector<std::pair<double, double>> hits;
    hits.reserve(cloud.points.size());

    for (const auto& point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        hits.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y));
    }

    publishOccupancyFromHits(get_publish_stamp(msg), hits);
}

void readParams()
{
    rosparam_get("occupancy_map/gridmap_enabled", gridmap_enabled, true);
    rosparam_get("occupancy_map/resolution", resolution, resolution);
    rosparam_get("occupancy_map/padding", padding, padding);
    rosparam_get("occupancy_map/max_size", max_size, max_size);
    rosparam_get("occupancy_map/accumulate", accumulate_map, accumulate_map);
    rosparam_get("occupancy_map/min_points_per_cell", min_points_per_cell, min_points_per_cell);
    rosparam_get("publish/use_current_time", use_current_time, true);
}

int main(int argc, char** argv)
{
#ifdef USE_ROS1
    ros::init(argc, argv, "cloud_to_occupancy");
    init_ros_node();
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    init_ros_node(rclcpp::Node::make_shared("cloud_to_occupancy"));
#endif

    readParams();

#ifdef USE_ROS1
    pubOccupancyGrid = create_publisher<OccupancyGridMsg>(output_topic, 1);
#elif defined(USE_ROS2)
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pubOccupancyGrid = create_publisher_qos<OccupancyGridMsg>(output_topic, map_qos);
#endif

    if (!gridmap_enabled) {
        ROS_PRINT_INFO("cloud_to_occupancy disabled");
    } else {
        auto subCloud = create_subscriber<PointCloud2Msg>(input_topic, 1, cloudCallback);
        ROS_PRINT_INFO(
            "cloud_to_occupancy subscribed to pointcloud %s, publishing %s in frame %s",
            input_topic.c_str(), output_topic.c_str(), map_frame.c_str());

        RateType rate(100.0);
        while (ros_ok()) {
            spin_once();
            rate.sleep();
        }
    }

    ros_shutdown();
    return 0;
}
