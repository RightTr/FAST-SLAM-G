#!/usr/bin/env python3

import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def default_tasks_dir():
    package_prefix = get_package_prefix("fast_lio_sam_g")
    workspace_root = os.path.dirname(os.path.dirname(package_prefix))
    source_tasks_dir = os.path.join(workspace_root, "src", "FAST-SLAM-G", "waypoints", "tasks")
    if os.path.isdir(os.path.dirname(source_tasks_dir)):
        return source_tasks_dir
    return os.path.join(get_package_share_directory("fast_lio_sam_g"), "waypoints", "tasks")


class WaypointTaskRunner(Node):
    def __init__(self):
        super().__init__("waypoint_task_runner")

        self.declare_parameter("task_name", "task")
        self.declare_parameter("task_file", "")
        self.declare_parameter("mode", "sequential")
        self.declare_parameter("navigate_to_pose_action", "navigate_to_pose")
        self.declare_parameter("navigate_through_poses_action", "navigate_through_poses")
        self.declare_parameter("marker_topic", "/waypoint_task_markers")
        self.declare_parameter("arrow_length", 0.65)
        self.declare_parameter("arrow_diameter", 0.08)
        self.declare_parameter("text_height", 0.28)
        self.declare_parameter("marker_z", 0.18)
        self.declare_parameter("text_z", 0.75)

        task_name = str(self.get_parameter("task_name").value)
        task_file = os.path.expanduser(str(self.get_parameter("task_file").value))
        if not task_file:
            task_file = os.path.join(default_tasks_dir(), f"{task_name}.yaml")

        self.task_file = task_file
        self.mode = str(self.get_parameter("mode").value).lower()
        self.navigate_to_pose_action = str(self.get_parameter("navigate_to_pose_action").value)
        self.navigate_through_poses_action = str(
            self.get_parameter("navigate_through_poses_action").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.arrow_length = float(self.get_parameter("arrow_length").value)
        self.arrow_diameter = float(self.get_parameter("arrow_diameter").value)
        self.text_height = float(self.get_parameter("text_height").value)
        self.marker_z = float(self.get_parameter("marker_z").value)
        self.text_z = float(self.get_parameter("text_z").value)

        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, marker_qos)

        self.waypoints, self.poses = self.load_task()
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, self.navigate_to_pose_action)
        self.nav_through_poses_client = ActionClient(
            self, NavigateThroughPoses, self.navigate_through_poses_action)

    def load_task(self):
        with open(self.task_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        waypoints = data.get("waypoints", []) if isinstance(data, dict) else []
        if not waypoints:
            raise RuntimeError(f"No waypoints in {self.task_file}")

        poses = []
        stamp = self.get_clock().now().to_msg()
        for waypoint in waypoints:
            pose_data = waypoint["pose"]
            position = pose_data["position"]
            orientation = pose_data["orientation"]

            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = str(waypoint.get("frame_id", "map"))
            msg.pose.position.x = float(position["x"])
            msg.pose.position.y = float(position["y"])
            msg.pose.position.z = float(position.get("z", 0.0))
            msg.pose.orientation.x = float(orientation.get("x", 0.0))
            msg.pose.orientation.y = float(orientation.get("y", 0.0))
            msg.pose.orientation.z = float(orientation.get("z", 0.0))
            msg.pose.orientation.w = float(orientation.get("w", 1.0))
            poses.append(msg)

        return waypoints, poses

    def run(self):
        self.get_logger().info(f"Loaded {len(self.poses)} task poses from {self.task_file}")
        self.publish_markers(active_index=0)

        if self.mode == "through":
            return self.run_through_poses()
        if self.mode != "sequential":
            raise RuntimeError("mode must be 'sequential' or 'through'")
        return self.run_sequential()

    def run_sequential(self):
        self.get_logger().info(
            f"Waiting for Nav2 action '{self.navigate_to_pose_action}'")
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                f"Action server '{self.navigate_to_pose_action}' is not available")

        for index, pose in enumerate(self.poses):
            self.publish_markers(active_index=index)
            name = self.waypoints[index].get("name", f"waypoint_{index + 1}")
            self.get_logger().info(
                f"Sending waypoint {index + 1}/{len(self.poses)}: {name}")

            goal = NavigateToPose.Goal()
            goal.pose = pose
            status = self.send_goal_and_wait(self.nav_to_pose_client, goal)
            if status != 4:
                self.publish_markers(active_index=index, failed_index=index)
                self.get_logger().error(f"Waypoint {name} failed, status={status}")
                return status

            self.publish_markers(active_index=index + 1, completed_count=index + 1)
            self.get_logger().info(f"Reached waypoint {index + 1}/{len(self.poses)}: {name}")

        self.publish_markers(active_index=-1, completed_count=len(self.poses))
        self.get_logger().info("Waypoint task finished")
        return 4

    def run_through_poses(self):
        self.get_logger().info(
            f"Waiting for Nav2 action '{self.navigate_through_poses_action}'")
        if not self.nav_through_poses_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                f"Action server '{self.navigate_through_poses_action}' is not available")

        goal = NavigateThroughPoses.Goal()
        goal.poses = self.poses
        self.get_logger().info(f"Sending waypoint task with {len(self.poses)} poses")
        status = self.send_goal_and_wait(self.nav_through_poses_client, goal)
        self.publish_markers(active_index=-1 if status == 4 else 0)
        self.get_logger().info(f"Waypoint task finished, status={status}")
        return status

    def send_goal_and_wait(self, client, goal):
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Waypoint task was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        return result.status if result is not None else None

    def publish_markers(self, active_index=-1, completed_count=0, failed_index=-1):
        markers = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        stamp = self.get_clock().now().to_msg()
        for index, waypoint in enumerate(self.waypoints):
            pose = waypoint["pose"]
            position = pose.get("position", {})
            orientation = pose.get("orientation", {})
            name = str(waypoint.get("name") or f"waypoint_{index + 1}")
            frame_id = str(waypoint.get("frame_id") or "map")

            x = float(position.get("x", 0.0))
            y = float(position.get("y", 0.0))
            base_z = float(position.get("z", 0.0)) + self.marker_z
            marker_id = index * 4

            if index == failed_index:
                point_color = (1.0, 0.0, 0.0, 1.0)
                arrow_color = (1.0, 0.0, 0.0, 1.0)
            elif index == active_index:
                point_color = (0.0, 1.0, 0.15, 1.0)
                arrow_color = (0.0, 1.0, 0.15, 1.0)
            elif index < completed_count:
                point_color = (0.3, 0.3, 0.3, 0.8)
                arrow_color = (0.3, 0.3, 0.3, 0.8)
            else:
                point_color = (0.05, 0.55, 1.0, 0.95)
                arrow_color = (1.0, 0.45, 0.05, 0.95)

            point_marker = self.make_marker(frame_id, stamp, "waypoint_points", marker_id)
            point_marker.type = Marker.SPHERE
            point_marker.pose.position.x = x
            point_marker.pose.position.y = y
            point_marker.pose.position.z = base_z
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = 0.22
            point_marker.scale.y = 0.22
            point_marker.scale.z = 0.08
            self.set_color(point_marker, point_color)
            markers.markers.append(point_marker)

            arrow_marker = self.make_marker(frame_id, stamp, "waypoint_directions", marker_id + 1)
            arrow_marker.type = Marker.ARROW
            arrow_marker.pose.position.x = x
            arrow_marker.pose.position.y = y
            arrow_marker.pose.position.z = base_z + 0.06
            arrow_marker.pose.orientation.x = float(orientation.get("x", 0.0))
            arrow_marker.pose.orientation.y = float(orientation.get("y", 0.0))
            arrow_marker.pose.orientation.z = float(orientation.get("z", 0.0))
            arrow_marker.pose.orientation.w = float(orientation.get("w", 1.0))
            arrow_marker.scale.x = self.arrow_length
            arrow_marker.scale.y = self.arrow_diameter
            arrow_marker.scale.z = self.arrow_diameter * 1.6
            self.set_color(arrow_marker, arrow_color)
            markers.markers.append(arrow_marker)

            shadow_marker = self.make_marker(frame_id, stamp, "waypoint_name_shadows", marker_id + 2)
            shadow_marker.type = Marker.TEXT_VIEW_FACING
            shadow_marker.pose.position.x = x + 0.03
            shadow_marker.pose.position.y = y - 0.03
            shadow_marker.pose.position.z = base_z + self.text_z - 0.01
            shadow_marker.pose.orientation.w = 1.0
            shadow_marker.scale.z = self.text_height
            self.set_color(shadow_marker, (0.0, 0.0, 0.0, 0.9))
            shadow_marker.text = name
            markers.markers.append(shadow_marker)

            text_marker = self.make_marker(frame_id, stamp, "waypoint_names", marker_id + 3)
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = base_z + self.text_z
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = self.text_height
            self.set_color(text_marker, (1.0, 0.02, 0.9, 1.0))
            text_marker.text = name
            markers.markers.append(text_marker)

        self.marker_pub.publish(markers)

    def make_marker(self, frame_id, stamp, namespace, marker_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        return marker

    def set_color(self, marker, color):
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointTaskRunner()
        status = node.run()
        sys.exit(0 if status == 4 else 1)
    except Exception as exc:
        print(f"waypoint_task_run failed: {exc}")
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
