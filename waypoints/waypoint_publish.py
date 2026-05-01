#!/usr/bin/env python3

import os

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__("waypoint_publisher")

        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_file = os.path.join(script_dir, "waypoints.yaml")

        self.declare_parameter("waypoints_file", default_file)
        self.declare_parameter("topic", "/goal_pose")
        self.declare_parameter("name", "")
        self.declare_parameter("index", -1)
        self.declare_parameter("repeat", 5)

        self.waypoints_file = os.path.expanduser(self.get_parameter("waypoints_file").value)
        self.topic = self.get_parameter("topic").value
        self.name = self.get_parameter("name").value
        self.index = self.get_parameter("index").value
        self.repeat = max(1, self.get_parameter("repeat").value)

        self.waypoint = self.load_waypoint()
        self.publisher = self.create_publisher(PoseStamped, self.topic, 10)
        self.publish_count = 0
        self.timer = self.create_timer(0.2, self.publish_waypoint)

    def load_waypoint(self):
        with open(self.waypoints_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        waypoints = data.get("waypoints") or []
        if not waypoints:
            raise RuntimeError(f"No waypoints in {self.waypoints_file}")

        if self.name:
            for waypoint in waypoints:
                if waypoint.get("name") == self.name:
                    return waypoint
            raise RuntimeError(f"Waypoint '{self.name}' not found")

        if self.index >= 0:
            if self.index >= len(waypoints):
                raise RuntimeError(f"Waypoint index {self.index} out of range")
            return waypoints[self.index]

        return waypoints[-1]

    def publish_waypoint(self):
        pose_data = self.waypoint["pose"]
        position = pose_data["position"]
        orientation = pose_data["orientation"]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.waypoint.get("frame_id", "map")
        msg.pose.position.x = float(position["x"])
        msg.pose.position.y = float(position["y"])
        msg.pose.position.z = float(position.get("z", 0.0))
        msg.pose.orientation.x = float(orientation["x"])
        msg.pose.orientation.y = float(orientation["y"])
        msg.pose.orientation.z = float(orientation["z"])
        msg.pose.orientation.w = float(orientation["w"])

        self.publisher.publish(msg)
        self.publish_count += 1

        if self.publish_count == 1:
            self.get_logger().info(
                f"Publishing {self.waypoint.get('name', 'waypoint')} to {self.topic}")

        if self.publish_count >= self.repeat:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WaypointPublisher()
        rclpy.spin(node)
    except Exception as exc:
        print(f"waypoint_publish failed: {exc}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
