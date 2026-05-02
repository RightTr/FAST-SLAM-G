#!/usr/bin/env python3

import datetime
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__("waypoint_recorder")

        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_output = os.path.join(script_dir, "waypoints.yaml")

        self.declare_parameter("odom_topic", "/OdometryGlobal")
        self.declare_parameter("output_file", default_output)
        self.declare_parameter("frame_id", "")
        self.declare_parameter("name", "")

        self.odom_topic = self.get_parameter("odom_topic").value
        self.output_file = os.path.expanduser(self.get_parameter("output_file").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.name = self.get_parameter("name").value
        self.saved = False

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile,
        )

        self.get_logger().info(f"Waiting for one odometry message on {self.odom_topic}")

    def odom_callback(self, msg):
        if self.saved:
            return

        self.saved = True
        pose = msg.pose.pose
        frame_id = self.frame_id or msg.header.frame_id or "map"
        waypoint_name = self.name or datetime.datetime.now().strftime("waypoint_%Y%m%d_%H%M%S")

        self.append_waypoint(waypoint_name, frame_id, pose)
        self.get_logger().info(f"Saved {waypoint_name} to {self.output_file}")
        rclpy.shutdown()

    def append_waypoint(self, name, frame_id, pose):
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        new_file = not os.path.exists(self.output_file) or os.path.getsize(self.output_file) == 0

        with open(self.output_file, "a", encoding="utf-8") as f:
            if new_file:
                f.write("waypoints:\n")

            f.write(f"  - name: {name}\n")
            f.write(f"    frame_id: {frame_id}\n")
            f.write("    pose:\n")
            f.write("      position:\n")
            f.write(f"        x: {pose.position.x:.9f}\n")
            f.write(f"        y: {pose.position.y:.9f}\n")
            f.write(f"        z: {pose.position.z:.9f}\n")
            f.write("      orientation:\n")
            f.write(f"        x: {pose.orientation.x:.9f}\n")
            f.write(f"        y: {pose.orientation.y:.9f}\n")
            f.write(f"        z: {pose.orientation.z:.9f}\n")
            f.write(f"        w: {pose.orientation.w:.9f}\n")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
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
