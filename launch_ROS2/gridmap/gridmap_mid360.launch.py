from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_ros1_yaml_as_params(yaml_file_path):
    with open(yaml_file_path, "r") as file:
        config = yaml.safe_load(file)

    def flatten_dict(d, parent_key="", sep="/"):
        items = []
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                items.extend(flatten_dict(v, new_key, sep=sep).items())
            else:
                items.append((new_key, v))
        return dict(items)

    return flatten_dict(config)


def generate_launch_description():
    package_share = get_package_share_directory("fast_lio_sam")
    rviz_cfg = os.path.join(package_share, "rviz_cfg", "grid_ros2.rviz")
    config_file = os.path.join(package_share, "config", "gridmap", "mid360.yaml")
    yaml_params = load_ros1_yaml_as_params(config_file)

    fast_lio_params = [
        {"sam_enable": True},
        {"feature_extract_enable": False},
        {"point_filter_num": 1},
        {"max_iteration": 3},
        {"filter_size_surf": 0.1},
        {"filter_size_map": 0.1},
        {"cube_side_length": 1000.0},
        {"runtime_pos_log_enable": False},
        yaml_params,
    ]

    fast_lio_sam = Node(
        package="fast_lio_sam",
        executable="fastlio_mapping",
        name="fast_lio_gridmap",
        output="screen",
        parameters=fast_lio_params,
    )

    cloud_to_occupancy = Node(
        package="fast_lio_sam",
        executable="cloud_to_occupancy",
        name="cloud_to_occupancy",
        output="screen",
        parameters=[yaml_params],
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        condition=IfCondition(str(yaml_params["pointcloud_to_laserscan/enabled"]).lower()),
        parameters=[{
            "queue_size": 10,
            "target_frame": yaml_params["pointcloud_to_laserscan/target_frame"],
            "transform_tolerance": yaml_params["pointcloud_to_laserscan/transform_tolerance"],
            "min_height": yaml_params["pointcloud_to_laserscan/min_height"],
            "max_height": yaml_params["pointcloud_to_laserscan/max_height"],
            "angle_min": yaml_params["pointcloud_to_laserscan/angle_min"],
            "angle_max": yaml_params["pointcloud_to_laserscan/angle_max"],
            "angle_increment": yaml_params["pointcloud_to_laserscan/angle_increment"],
            "scan_time": yaml_params["pointcloud_to_laserscan/scan_time"],
            "range_min": yaml_params["pointcloud_to_laserscan/range_min"],
            "range_max": yaml_params["pointcloud_to_laserscan/range_max"],
            "use_inf": yaml_params["pointcloud_to_laserscan/use_inf"],
            "inf_epsilon": yaml_params["pointcloud_to_laserscan/inf_epsilon"],
        }],
        remappings=[
            ("cloud_in", yaml_params["pointcloud_to_laserscan/input_topic"]),
            ("scan", yaml_params["pointcloud_to_laserscan/output_topic"]),
        ],
    )

    fast_lio_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
    )

    return LaunchDescription([
        fast_lio_sam,
        cloud_to_occupancy,
        pointcloud_to_laserscan,
        fast_lio_rviz,
    ])
