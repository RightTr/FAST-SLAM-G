import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = get_package_share_directory("fast_lio_sam_g")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    publish_waypoints = LaunchConfiguration("publish_waypoints")
    waypoints_file = LaunchConfiguration("waypoints_file")
    waypoint_marker_topic = LaunchConfiguration("waypoint_marker_topic")
    launch_task_dispatcher = LaunchConfiguration("launch_task_dispatcher")
    task_command_topic = LaunchConfiguration("task_command_topic")
    task_status_topic = LaunchConfiguration("task_status_topic")
    task_result_flag_topic = LaunchConfiguration("task_result_flag_topic")
    map_file = LaunchConfiguration("map")
    map_source = LaunchConfiguration("map_source")
    map_edits_file = LaunchConfiguration("map_edits_file")
    export_map_yaml = LaunchConfiguration("export_map_yaml")
    map_edit_mode = LaunchConfiguration("map_edit_mode")
    brush_radius = LaunchConfiguration("brush_radius")
    offline_map_topic = LaunchConfiguration("offline_map_topic")
    online_map_topic = LaunchConfiguration("online_map_topic")
    nav2_static_map_topic = PythonExpression([
        "'", offline_map_topic, "' if '", map_source, "' == 'offline' else '",
        online_map_topic, "'"
    ])

    package_prefix = get_package_prefix("fast_lio_sam_g")
    workspace_root = os.path.dirname(os.path.dirname(package_prefix))
    source_map_dir = os.path.join(workspace_root, "src", "FAST-SLAM-G", "map")
    source_map_file = os.path.join(source_map_dir, "edited_map.yaml")
    source_edits_file = os.path.join(source_map_dir, "map_edits.yaml")
    installed_map_file = os.path.join(package_share, "map", "edited_map.yaml")
    installed_edits_file = os.path.join(package_share, "map", "map_edits.yaml")
    default_map_file = source_map_file if os.path.exists(source_map_file) else installed_map_file
    default_edits_file = source_edits_file if os.path.exists(source_edits_file) else installed_edits_file
    source_waypoints_file = os.path.join(
        workspace_root, "src", "FAST-SLAM-G", "waypoints", "waypoints.yaml")
    installed_waypoints_file = os.path.join(package_share, "waypoints", "waypoints.yaml")
    default_waypoints_file = (
        source_waypoints_file if os.path.exists(source_waypoints_file)
        else installed_waypoints_file
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

    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    cmd_vel_remappings = [("cmd_vel", "cmd_vel_nav")]
    velocity_smoother_remappings = [
        ("cmd_vel", "cmd_vel_nav"),
        ("cmd_vel_smoothed", "cmd_vel"),
    ]

    lifecycle_nodes_online_map = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]
    lifecycle_nodes_offline_map = [
        "map_server",
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_share, "config", "nav2", "mid360.yaml"),
                description="Full path to the Nav2 parameters file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock if true.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically configure and activate Nav2 lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz2 with the Nav2 visualization config.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(package_share, "rviz_cfg", "nav2_ros2.rviz"),
                description="Full path to the RViz2 config file.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map_file,
                description="Full path to the offline map YAML loaded by Nav2 map_server.",
            ),
            DeclareLaunchArgument(
                "map_source",
                default_value="offline",
                description="Map source for Nav2: offline starts map_server, online expects /map from gridmap.",
            ),
            DeclareLaunchArgument(
                "map_edits_file",
                default_value=default_edits_file,
                description="YAML file that stores manual occupancy-grid edits for online map mode.",
            ),
            DeclareLaunchArgument(
                "export_map_yaml",
                default_value=default_map_file,
                description="Map-server YAML path for exporting the edited online occupancy grid.",
            ),
            DeclareLaunchArgument(
                "map_edit_mode",
                default_value="free",
                description="Online map click edit mode: free, occupied, or unknown.",
            ),
            DeclareLaunchArgument(
                "brush_radius",
                default_value="0.20",
                description="Online map editing brush radius in meters.",
            ),
            DeclareLaunchArgument(
                "offline_map_topic",
                default_value="/nav2_map",
                description="Topic used by Nav2 for the offline map_server map.",
            ),
            DeclareLaunchArgument(
                "online_map_topic",
                default_value="/nav2_online_map",
                description="Topic used by Nav2 for the edited online gridmap.",
            ),
            DeclareLaunchArgument(
                "publish_waypoints",
                default_value="true",
                description="Publish waypoint markers for RViz.",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=default_waypoints_file,
                description="Waypoint YAML file to show in RViz.",
            ),
            DeclareLaunchArgument(
                "waypoint_marker_topic",
                default_value="/waypoints_markers",
                description="MarkerArray topic used to display waypoints in RViz.",
            ),
            DeclareLaunchArgument(
                "launch_task_dispatcher",
                default_value="true",
                description="Start the numeric waypoint task dispatcher.",
            ),
            DeclareLaunchArgument(
                "task_command_topic",
                default_value="/waypoint_task_id",
                description="std_msgs/Int32 topic used to trigger waypoint tasks.",
            ),
            DeclareLaunchArgument(
                "task_status_topic",
                default_value="/waypoint_task_status",
                description="std_msgs/String topic used to publish waypoint task progress and result.",
            ),
            DeclareLaunchArgument(
                "task_result_flag_topic",
                default_value="/waypoint_task_result",
                description="std_msgs/Int8 topic used to publish final waypoint task result: 1 success, 0 failure.",
            ),
            Node(
                condition=IfCondition(PythonExpression(["'", map_source, "' == 'offline'"])),
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"yaml_filename": map_file},
                ],
                remappings=[("map", offline_map_topic)],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                condition=IfCondition(PythonExpression(["'", map_source, "' == 'online'"])),
                package="fast_lio_sam_g",
                executable="occupancy_map_editor.py",
                name="occupancy_map_editor",
                output="screen",
                parameters=[
                    {"input_map_topic": "/map"},
                    {"output_map_topic": online_map_topic},
                    {"clicked_point_topic": "/clicked_point"},
                    {"edits_file": map_edits_file},
                    {"export_map_yaml": export_map_yaml},
                    {"mode": map_edit_mode},
                    {"brush_radius": brush_radius},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remappings + cmd_vel_remappings,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remappings,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remappings,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remappings + cmd_vel_remappings,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remappings,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remappings,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[configured_params],
                remappings=tf_remappings + velocity_smoother_remappings,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                condition=IfCondition(PythonExpression(["'", map_source, "' == 'offline'"])),
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes_offline_map},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                condition=UnlessCondition(PythonExpression(["'", map_source, "' == 'offline'"])),
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes_online_map},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                condition=IfCondition(publish_waypoints),
                package="fast_lio_sam_g",
                executable="waypoint_markers.py",
                name="waypoint_markers",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"waypoints_file": waypoints_file},
                    {"marker_topic": waypoint_marker_topic},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                condition=IfCondition(launch_task_dispatcher),
                package="fast_lio_sam_g",
                executable="waypoint_task_dispatcher.py",
                name="waypoint_task_dispatcher",
                output="screen",
                parameters=[
                    {"command_topic": task_command_topic},
                    {"status_topic": task_status_topic},
                    {"result_flag_topic": task_result_flag_topic},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                condition=IfCondition(launch_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
