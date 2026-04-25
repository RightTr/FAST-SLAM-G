#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class CloudToOccupancyNode : public rclcpp::Node
{
public:
    CloudToOccupancyNode()
    : Node("cloud_to_occupancy")
    {
        enabled_ = this->declare_parameter<bool>("occupancy_map/enabled", true);
        input_topic_ = this->declare_parameter<std::string>(
            "occupancy_map/input_topic", "lio_sam/mapping/cloud_global");
        output_topic_ = this->declare_parameter<std::string>(
            "occupancy_map/output_topic", "/map");

        const std::string configured_map_frame =
            this->declare_parameter<std::string>("occupancy_map/map_frame", "");
        const std::string lio_map_frame =
            this->declare_parameter<std::string>("lio_sam/mapFrame", "map");
        map_frame_ = configured_map_frame.empty() ? lio_map_frame : configured_map_frame;

        resolution_ = this->declare_parameter<double>("occupancy_map/resolution", 0.10);
        padding_ = this->declare_parameter<double>("occupancy_map/padding", 5.0);
        obstacle_min_height_ =
            this->declare_parameter<double>("occupancy_map/obstacle_min_height", -0.30);
        obstacle_max_height_ =
            this->declare_parameter<double>("occupancy_map/obstacle_max_height", 1.50);
        inflation_radius_ =
            this->declare_parameter<double>("occupancy_map/inflation_radius", 0.20);
        min_points_per_cell_ =
            this->declare_parameter<int>("occupancy_map/min_points_per_cell", 1);
        fill_free_space_ =
            this->declare_parameter<bool>("occupancy_map/fill_free_space", true);

        const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            output_topic_, map_qos);

        if (!enabled_) {
            RCLCPP_INFO(get_logger(), "occupancy projection disabled");
            return;
        }

        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            std::bind(&CloudToOccupancyNode::cloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "projecting %s to %s in frame %s (resolution %.3f m)",
            input_topic_.c_str(),
            output_topic_.c_str(),
            map_frame_.c_str(),
            resolution_);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        std::vector<std::size_t> occupied_indices;
        occupied_indices.reserve(cloud.points.size());

        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();

        for (const auto & point : cloud.points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            if (point.z < obstacle_min_height_ || point.z > obstacle_max_height_) {
                continue;
            }

            min_x = std::min(min_x, static_cast<double>(point.x));
            min_y = std::min(min_y, static_cast<double>(point.y));
            max_x = std::max(max_x, static_cast<double>(point.x));
            max_y = std::max(max_y, static_cast<double>(point.y));
        }

        if (!std::isfinite(min_x) || !std::isfinite(min_y) ||
            !std::isfinite(max_x) || !std::isfinite(max_y)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "no points remain after height filtering");
            return;
        }

        min_x -= padding_;
        min_y -= padding_;
        max_x += padding_;
        max_y += padding_;

        const auto width = static_cast<std::uint32_t>(
            std::max(1.0, std::ceil((max_x - min_x) / resolution_)));
        const auto height = static_cast<std::uint32_t>(
            std::max(1.0, std::ceil((max_y - min_y) / resolution_)));

        std::vector<int> hit_count(width * height, 0);
        for (const auto & point : cloud.points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            if (point.z < obstacle_min_height_ || point.z > obstacle_max_height_) {
                continue;
            }

            const int cell_x = static_cast<int>(std::floor((point.x - min_x) / resolution_));
            const int cell_y = static_cast<int>(std::floor((point.y - min_y) / resolution_));
            if (cell_x < 0 || cell_y < 0 ||
                cell_x >= static_cast<int>(width) || cell_y >= static_cast<int>(height)) {
                continue;
            }

            const std::size_t index =
                static_cast<std::size_t>(cell_y) * width + static_cast<std::size_t>(cell_x);
            hit_count[index] += 1;
        }

        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = msg->header.stamp;
        grid.header.frame_id = map_frame_;
        grid.info.map_load_time = msg->header.stamp;
        grid.info.resolution = static_cast<float>(resolution_);
        grid.info.width = width;
        grid.info.height = height;
        grid.info.origin.position.x = min_x;
        grid.info.origin.position.y = min_y;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(width * height, fill_free_space_ ? 0 : -1);

        for (std::size_t idx = 0; idx < hit_count.size(); ++idx) {
            if (hit_count[idx] >= min_points_per_cell_) {
                grid.data[idx] = 100;
                occupied_indices.push_back(idx);
            }
        }

        inflateOccupiedCells(width, height, occupied_indices, grid.data);
        map_publisher_->publish(grid);
    }

    void inflateOccupiedCells(
        std::uint32_t width,
        std::uint32_t height,
        const std::vector<std::size_t>& occupied_indices,
        std::vector<int8_t>& grid_data) const
    {
        const int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
        if (inflation_cells <= 0 || occupied_indices.empty()) {
            return;
        }

        std::vector<int8_t> inflated = grid_data;
        for (const std::size_t idx : occupied_indices) {
            const int cell_x = static_cast<int>(idx % width);
            const int cell_y = static_cast<int>(idx / width);

            for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                    if (dx * dx + dy * dy > inflation_cells * inflation_cells) {
                        continue;
                    }

                    const int nx = cell_x + dx;
                    const int ny = cell_y + dy;
                    if (nx < 0 || ny < 0 ||
                        nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) {
                        continue;
                    }

                    inflated[static_cast<std::size_t>(ny) * width +
                             static_cast<std::size_t>(nx)] = 100;
                }
            }
        }

        grid_data.swap(inflated);
    }

    bool enabled_{true};
    bool fill_free_space_{true};
    int min_points_per_cell_{1};
    double resolution_{0.1};
    double padding_{5.0};
    double obstacle_min_height_{-0.3};
    double obstacle_max_height_{1.5};
    double inflation_radius_{0.2};
    std::string input_topic_;
    std::string output_topic_;
    std::string map_frame_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudToOccupancyNode>());
    rclcpp::shutdown();
    return 0;
}
