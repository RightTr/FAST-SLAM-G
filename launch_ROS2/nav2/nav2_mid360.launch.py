import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def source_path(*parts):
    package_prefix = get_package_prefix("fast_lio_sam_g")
    workspace_root = os.path.dirname(os.path.dirname(package_prefix))
    return os.path.join(workspace_root, "src", "FAST-SLAM-G", *parts)


def existing_path(source_file, installed_file):
    return source_file if os.path.exists(source_file) else installed_file


def nav2_node(package, executable, configured_params, log_level, remappings=None, name=None):
    return Node(
        package=package,
        executable=executable,
        name=name,
        output="screen",
        parameters=[configured_params],
        remappings=remappings or [],
        arguments=["--ros-args", "--log-level", log_level],
    )


def lifecycle_manager(name, nodes, use_sim_time, log_level, condition):
    return Node(
        condition=condition,
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name=name,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": nodes},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )


def generate_launch_description():
    package_share = get_package_share_directory("fast_lio_sam_g")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    launch_rviz = LaunchConfiguration("launch_rviz")
    map_source = LaunchConfiguration("map_source")
    map_file = LaunchConfiguration("map")

    offline_map_topic = "/nav2_map"
    online_map_topic = "/nav2_online_map"
    nav2_static_map_topic = PythonExpression([
        f"'{offline_map_topic}' if '", map_source, f"' == 'offline' else '{online_map_topic}'"
    ])

    default_map = existing_path(
        source_path("gridmap", "map", "edited_map.yaml"),
        os.path.join(package_share, "gridmap", "map", "edited_map.yaml"),
    )
    default_waypoints = existing_path(
        source_path("waypoints", "points", "waypoints.yaml"),
        os.path.join(package_share, "waypoints", "points", "waypoints.yaml"),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "global_costmap.global_costmap.ros__parameters.static_layer.map_topic":
                    nav2_static_map_topic,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    nav2_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]
    offline_lifecycle_nodes = ["map_server"] + nav2_nodes
    online_lifecycle_nodes = nav2_nodes

    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    cmd_vel_remaps = [("cmd_vel", "cmd_vel_nav")]
    smoother_remaps = [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]
    offline_condition = IfCondition(PythonExpression(["'", map_source, "' == 'offline'"]))
    online_condition = IfCondition(PythonExpression(["'", map_source, "' == 'online'"]))
    not_offline_condition = IfCondition(PythonExpression(["'", map_source, "' != 'offline'"]))

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(package_share, "config", "nav2", "mid360.yaml"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("map_source", default_value="offline"),
        DeclareLaunchArgument("map", default_value=default_map),

        Node(
            condition=offline_condition,
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}, {"yaml_filename": map_file}],
            remappings=[("map", offline_map_topic)],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            condition=online_condition,
            package="fast_lio_sam_g",
            executable="occupancy_map_editor.py",
            name="occupancy_map_editor",
            output="screen",
            parameters=[
                {"input_map_topic": "/map"},
                {"output_map_topic": online_map_topic},
                {"clicked_point_topic": "/clicked_point"},
                {"edits_file": ""},
                {"export_map_yaml": default_map},
                {"mode": "free"},
                {"brush_radius": 0.20},
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        nav2_node("nav2_controller", "controller_server", configured_params, log_level, tf_remaps + cmd_vel_remaps),
        nav2_node("nav2_smoother", "smoother_server", configured_params, log_level, tf_remaps),
        nav2_node("nav2_planner", "planner_server", configured_params, log_level, tf_remaps, "planner_server"),
        nav2_node("nav2_behaviors", "behavior_server", configured_params, log_level, tf_remaps + cmd_vel_remaps, "behavior_server"),
        nav2_node("nav2_bt_navigator", "bt_navigator", configured_params, log_level, tf_remaps, "bt_navigator"),
        nav2_node("nav2_waypoint_follower", "waypoint_follower", configured_params, log_level, tf_remaps, "waypoint_follower"),
        nav2_node("nav2_velocity_smoother", "velocity_smoother", configured_params, log_level, tf_remaps + smoother_remaps, "velocity_smoother"),
        lifecycle_manager("lifecycle_manager_navigation", offline_lifecycle_nodes, use_sim_time, log_level, offline_condition),
        lifecycle_manager("lifecycle_manager_navigation", online_lifecycle_nodes, use_sim_time, log_level, not_offline_condition),
        Node(
            package="fast_lio_sam_g",
            executable="waypoint_markers.py",
            name="waypoint_markers",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"waypoints_file": default_waypoints},
                {"marker_topic": "/waypoints_markers"},
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="fast_lio_sam_g",
            executable="waypoint_task_dispatcher.py",
            name="waypoint_task_dispatcher",
            output="screen",
            parameters=[
                {"command_topic": "/waypoint_task_id"},
                {"status_topic": "/waypoint_task_status"},
                {"result_flag_topic": "/waypoint_task_result"},
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            condition=IfCondition(launch_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(package_share, "rviz_cfg", "nav2_ros2.rviz")],
        ),
    ])
