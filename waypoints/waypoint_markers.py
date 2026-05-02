#!/usr/bin/env python3

import math
import os

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q["w"] * q["z"] + q["x"] * q["y"])
    cosy_cosp = 1.0 - 2.0 * (q["y"] * q["y"] + q["z"] * q["z"])
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    half_yaw = 0.5 * yaw
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half_yaw),
        "w": math.cos(half_yaw),
    }


class WaypointMarkers(Node):
    def __init__(self):
        super().__init__("waypoint_markers")

        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_file = os.path.join(script_dir, "waypoints.yaml")

        self.declare_parameter("waypoints_file", default_file)
        self.declare_parameter("marker_topic", "/waypoints_markers")
        self.declare_parameter("reload_period", 1.0)
        self.declare_parameter("arrow_length", 0.65)
        self.declare_parameter("arrow_diameter", 0.08)
        self.declare_parameter("text_height", 0.28)
        self.declare_parameter("marker_z", 0.18)
        self.declare_parameter("text_z", 0.75)

        self.waypoints_file = os.path.expanduser(self.get_parameter("waypoints_file").value)
        self.reload_period = float(self.get_parameter("reload_period").value)
        self.arrow_length = float(self.get_parameter("arrow_length").value)
        self.arrow_diameter = float(self.get_parameter("arrow_diameter").value)
        self.text_height = float(self.get_parameter("text_height").value)
        self.marker_z = float(self.get_parameter("marker_z").value)
        self.text_z = float(self.get_parameter("text_z").value)
        marker_topic = self.get_parameter("marker_topic").value

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(MarkerArray, marker_topic, qos)
        self.last_mtime = None
        self.timer = self.create_timer(self.reload_period, self.publish_if_changed)

        self.get_logger().info(f"Publishing waypoint markers from {self.waypoints_file}")
        self.publish_if_changed(force=True)

    def publish_if_changed(self, force=False):
        try:
            mtime = os.path.getmtime(self.waypoints_file)
        except OSError as exc:
            if force:
                self.get_logger().warn(f"Cannot read waypoints file: {exc}")
            return

        if not force and self.last_mtime == mtime:
            return

        self.last_mtime = mtime
        try:
            waypoints = self.load_waypoints()
        except Exception as exc:
            self.get_logger().error(f"Failed to load waypoint markers: {exc}")
            return

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL

        markers = MarkerArray()
        markers.markers.append(delete_all)
        markers.markers.extend(self.create_markers(waypoints))
        self.publisher.publish(markers)
        self.get_logger().info(f"Published {len(waypoints)} waypoint markers")

    def load_waypoints(self):
        with open(self.waypoints_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or []

        if isinstance(data, dict):
            waypoints = data.get("waypoints") or []
        elif isinstance(data, list):
            waypoints = data
        else:
            raise RuntimeError("waypoints yaml must be a list or contain a 'waypoints' list")

        valid_waypoints = []
        for waypoint in waypoints:
            if not isinstance(waypoint, dict):
                continue
            if "pose" not in waypoint:
                continue
            valid_waypoints.append(waypoint)
        return valid_waypoints

    def create_markers(self, waypoints):
        markers = []
        stamp = self.get_clock().now().to_msg()

        for index, waypoint in enumerate(waypoints):
            pose = waypoint["pose"]
            position = pose.get("position", {})
            orientation = pose.get("orientation", {})
            name = str(waypoint.get("name") or f"waypoint_{index}")
            frame_id = str(waypoint.get("frame_id") or "map")

            x = float(position.get("x", 0.0))
            y = float(position.get("y", 0.0))
            base_z = float(position.get("z", 0.0)) + self.marker_z
            q = {
                "x": float(orientation.get("x", 0.0)),
                "y": float(orientation.get("y", 0.0)),
                "z": float(orientation.get("z", 0.0)),
                "w": float(orientation.get("w", 1.0)),
            }
            flat_q = yaw_to_quaternion(yaw_from_quaternion(q))

            marker_id = index * 4

            point_marker = self.make_base_marker(frame_id, stamp, "waypoint_points", marker_id)
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
            markers.append(point_marker)

            arrow_marker = self.make_base_marker(frame_id, stamp, "waypoint_directions", marker_id + 1)
            arrow_marker.type = Marker.ARROW
            arrow_marker.pose.position.x = x
            arrow_marker.pose.position.y = y
            arrow_marker.pose.position.z = base_z + 0.06
            arrow_marker.pose.orientation.x = flat_q["x"]
            arrow_marker.pose.orientation.y = flat_q["y"]
            arrow_marker.pose.orientation.z = flat_q["z"]
            arrow_marker.pose.orientation.w = flat_q["w"]
            arrow_marker.scale.x = self.arrow_length
            arrow_marker.scale.y = self.arrow_diameter
            arrow_marker.scale.z = self.arrow_diameter * 1.6
            arrow_marker.color.r = 1.0
            arrow_marker.color.g = 0.45
            arrow_marker.color.b = 0.05
            arrow_marker.color.a = 0.95
            markers.append(arrow_marker)

            text_shadow = self.make_base_marker(frame_id, stamp, "waypoint_name_shadows", marker_id + 2)
            text_shadow.type = Marker.TEXT_VIEW_FACING
            text_shadow.pose.position.x = x + 0.03
            text_shadow.pose.position.y = y - 0.03
            text_shadow.pose.position.z = base_z + self.text_z - 0.01
            text_shadow.pose.orientation.w = 1.0
            text_shadow.scale.z = self.text_height
            text_shadow.color.r = 0.0
            text_shadow.color.g = 0.0
            text_shadow.color.b = 0.0
            text_shadow.color.a = 0.9
            text_shadow.text = name
            markers.append(text_shadow)

            text_marker = self.make_base_marker(frame_id, stamp, "waypoint_names", marker_id + 3)
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
            markers.append(text_marker)

        return markers

    def make_base_marker(self, frame_id, stamp, namespace, marker_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
