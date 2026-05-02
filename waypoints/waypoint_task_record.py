#!/usr/bin/env python3

import datetime
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

try:
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
except ImportError:
    get_package_prefix = None
    get_package_share_directory = None


def default_tasks_dir():
    if get_package_prefix is not None and get_package_share_directory is not None:
        try:
            package_prefix = get_package_prefix("fast_lio_sam_g")
            workspace_root = os.path.dirname(os.path.dirname(package_prefix))
            source_tasks_dir = os.path.join(
                workspace_root, "src", "FAST-SLAM-G", "waypoints", "tasks")
            if os.path.isdir(os.path.dirname(source_tasks_dir)):
                return source_tasks_dir
            return os.path.join(get_package_share_directory("fast_lio_sam_g"), "waypoints", "tasks")
        except Exception:
            pass

    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "tasks")


def yaw_to_quaternion(yaw):
    half_yaw = 0.5 * yaw
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half_yaw),
        "w": math.cos(half_yaw),
    }


def pose_to_dict(pose):
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


class WaypointTaskRecorder(Node):
    def __init__(self):
        super().__init__("waypoint_task_recorder")

        default_task_name = datetime.datetime.now().strftime("task_%Y%m%d_%H%M%S")
        self.declare_parameter("task_name", default_task_name)
        self.declare_parameter("task_file", "")
        self.declare_parameter("goal_pose_topic", "/waypoint_task_pose")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("record_goal_pose", True)
        self.declare_parameter("record_clicked_point", False)
        self.declare_parameter("clear_existing", True)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("min_separation", 0.05)
        self.declare_parameter("marker_topic", "/waypoint_task_markers")
        self.declare_parameter("arrow_length", 0.65)
        self.declare_parameter("arrow_diameter", 0.08)
        self.declare_parameter("text_height", 0.28)
        self.declare_parameter("marker_z", 0.18)
        self.declare_parameter("text_z", 0.75)

        self.task_name = str(self.get_parameter("task_name").value)
        self.task_file = os.path.expanduser(str(self.get_parameter("task_file").value))
        if not self.task_file:
            self.task_file = os.path.join(default_tasks_dir(), f"{self.task_name}.yaml")
        self.goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        self.clicked_point_topic = str(self.get_parameter("clicked_point_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.min_separation = float(self.get_parameter("min_separation").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.arrow_length = float(self.get_parameter("arrow_length").value)
        self.arrow_diameter = float(self.get_parameter("arrow_diameter").value)
        self.text_height = float(self.get_parameter("text_height").value)
        self.marker_z = float(self.get_parameter("marker_z").value)
        self.text_z = float(self.get_parameter("text_z").value)
        self.waypoints = []

        if not bool(self.get_parameter("clear_existing").value):
            self.waypoints = self.load_existing_waypoints()

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        if bool(self.get_parameter("record_goal_pose").value):
            self.goal_sub = self.create_subscription(
                PoseStamped, self.goal_pose_topic, self.goal_pose_callback, qos)
        if bool(self.get_parameter("record_clicked_point").value):
            self.point_sub = self.create_subscription(
                PointStamped, self.clicked_point_topic, self.clicked_point_callback, qos)

        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, marker_qos)

        self.write_task_file()
        self.publish_markers()
        self.get_logger().info(
            f"Recording waypoint task '{self.task_name}' to {self.task_file}")
        self.get_logger().info(
            f"Use RViz 2D Goal Pose on {self.goal_pose_topic}; "
            f"Publish Point recording is {'enabled' if bool(self.get_parameter('record_clicked_point').value) else 'disabled'}")
        self.get_logger().info(f"Showing recorded task markers on {self.marker_topic}")

    def load_existing_waypoints(self):
        if not os.path.exists(self.task_file):
            return []

        with open(self.task_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        waypoints = data.get("waypoints", []) if isinstance(data, dict) else []
        if not isinstance(waypoints, list):
            return []
        return waypoints

    def goal_pose_callback(self, msg):
        pose = pose_to_dict(msg.pose)
        frame_id = msg.header.frame_id or self.frame_id
        self.append_waypoint(frame_id, pose, "goal")

    def clicked_point_callback(self, msg):
        frame_id = msg.header.frame_id or self.frame_id
        pose = {
            "position": {
                "x": float(msg.point.x),
                "y": float(msg.point.y),
                "z": float(msg.point.z),
            },
            "orientation": yaw_to_quaternion(0.0),
        }
        self.append_waypoint(frame_id, pose, "point")
        self.update_point_yaws()

    def append_waypoint(self, frame_id, pose, source):
        if self.is_duplicate(pose):
            return

        index = len(self.waypoints) + 1
        self.waypoints.append({
            "name": f"{self.task_name}_{index:03d}",
            "frame_id": frame_id,
            "pose": pose,
        })
        self.write_task_file()
        self.publish_markers()
        position = pose["position"]
        self.get_logger().info(
            f"Saved {source} {index}: x={position['x']:.3f}, y={position['y']:.3f}")

    def is_duplicate(self, pose):
        if not self.waypoints:
            return False

        last = self.waypoints[-1].get("pose", {}).get("position", {})
        dx = float(pose["position"]["x"]) - float(last.get("x", 0.0))
        dy = float(pose["position"]["y"]) - float(last.get("y", 0.0))
        return math.hypot(dx, dy) < self.min_separation

    def update_point_yaws(self):
        if len(self.waypoints) < 2:
            return

        for index in range(len(self.waypoints) - 1):
            current = self.waypoints[index]["pose"]["position"]
            nxt = self.waypoints[index + 1]["pose"]["position"]
            yaw = math.atan2(
                float(nxt["y"]) - float(current["y"]),
                float(nxt["x"]) - float(current["x"]),
            )
            self.waypoints[index]["pose"]["orientation"] = yaw_to_quaternion(yaw)

        self.waypoints[-1]["pose"]["orientation"] = self.waypoints[-2]["pose"]["orientation"]
        self.write_task_file()
        self.publish_markers()

    def write_task_file(self):
        os.makedirs(os.path.dirname(self.task_file), exist_ok=True)
        data = {
            "task_name": self.task_name,
            "updated": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "waypoints": self.waypoints,
        }
        with open(self.task_file, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def publish_markers(self):
        markers = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        stamp = self.get_clock().now().to_msg()
        for index, waypoint in enumerate(self.waypoints):
            pose = waypoint["pose"]
            position = pose.get("position", {})
            orientation = pose.get("orientation", {})
            name = str(waypoint.get("name") or f"{self.task_name}_{index + 1:03d}")
            frame_id = str(waypoint.get("frame_id") or self.frame_id)

            x = float(position.get("x", 0.0))
            y = float(position.get("y", 0.0))
            base_z = float(position.get("z", 0.0)) + self.marker_z
            marker_id = index * 4

            point_marker = self.make_marker(frame_id, stamp, "waypoint_points", marker_id)
            point_marker.type = Marker.SPHERE
            point_marker.pose.position.x = x
            point_marker.pose.position.y = y
            point_marker.pose.position.z = base_z
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = 0.22
            point_marker.scale.y = 0.22
            point_marker.scale.z = 0.08
            point_marker.color.r = 0.05
            point_marker.color.g = 0.55
            point_marker.color.b = 1.0
            point_marker.color.a = 0.95
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
            arrow_marker.color.r = 1.0
            arrow_marker.color.g = 0.45
            arrow_marker.color.b = 0.05
            arrow_marker.color.a = 0.95
            markers.markers.append(arrow_marker)

            shadow_marker = self.make_marker(frame_id, stamp, "waypoint_name_shadows", marker_id + 2)
            shadow_marker.type = Marker.TEXT_VIEW_FACING
            shadow_marker.pose.position.x = x + 0.03
            shadow_marker.pose.position.y = y - 0.03
            shadow_marker.pose.position.z = base_z + self.text_z - 0.01
            shadow_marker.pose.orientation.w = 1.0
            shadow_marker.scale.z = self.text_height
            shadow_marker.color.r = 0.0
            shadow_marker.color.g = 0.0
            shadow_marker.color.b = 0.0
            shadow_marker.color.a = 0.9
            shadow_marker.text = name
            markers.markers.append(shadow_marker)

            text_marker = self.make_marker(frame_id, stamp, "waypoint_names", marker_id + 3)
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = base_z + self.text_z
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = self.text_height
            text_marker.color.r = 1.0
            text_marker.color.g = 0.02
            text_marker.color.b = 0.9
            text_marker.color.a = 1.0
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

    def finish(self):
        self.write_task_file()
        self.publish_markers()
        self.get_logger().info(
            f"Finished task '{self.task_name}': {len(self.waypoints)} waypoints saved to {self.task_file}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointTaskRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
