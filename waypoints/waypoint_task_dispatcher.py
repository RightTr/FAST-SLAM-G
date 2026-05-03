#!/usr/bin/env python3

import os
import sys
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8, Int32, String
from visualization_msgs.msg import Marker, MarkerArray


def default_tasks_dir():
    package_prefix = get_package_prefix("fast_lio_sam_g")
    workspace_root = os.path.dirname(os.path.dirname(package_prefix))
    source_tasks_dir = os.path.join(workspace_root, "src", "FAST-SLAM-G", "waypoints", "tasks")
    if os.path.isdir(os.path.dirname(source_tasks_dir)):
        return source_tasks_dir
    return os.path.join(get_package_share_directory("fast_lio_sam_g"), "waypoints", "tasks")


class WaypointTaskDispatcher(Node):
    def __init__(self):
        super().__init__("waypoint_task_dispatcher")

        self.declare_parameter("command_topic", "/waypoint_task_id")
        self.declare_parameter("task_dir", default_tasks_dir())
        self.declare_parameter("task_map", [
            "1:restaurant_start.yaml",
            "2:table2trash.yaml",
            "3:trash2table.yaml",
            "4:table2binx.yaml",
            "5:binx2table.yaml",
            "6:table2guizi.yaml",
            "7:guizi2table.yaml",
            "8:table2desk.yaml",
            "9:desk2table.yaml",
        ])
        self.declare_parameter("navigate_to_pose_action", "navigate_to_pose")
        self.declare_parameter("marker_topic", "/waypoint_task_markers")
        self.declare_parameter("status_topic", "/waypoint_task_status")
        self.declare_parameter("result_flag_topic", "/waypoint_task_result")
        self.declare_parameter("arrow_length", 0.65)
        self.declare_parameter("arrow_diameter", 0.08)
        self.declare_parameter("text_height", 0.28)
        self.declare_parameter("marker_z", 0.18)
        self.declare_parameter("text_z", 0.75)

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.task_dir = os.path.expanduser(str(self.get_parameter("task_dir").value))
        self.task_map = self.parse_task_map(self.get_parameter("task_map").value)
        self.navigate_to_pose_action = str(self.get_parameter("navigate_to_pose_action").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.result_flag_topic = str(self.get_parameter("result_flag_topic").value)
        self.arrow_length = float(self.get_parameter("arrow_length").value)
        self.arrow_diameter = float(self.get_parameter("arrow_diameter").value)
        self.text_height = float(self.get_parameter("text_height").value)
        self.marker_z = float(self.get_parameter("marker_z").value)
        self.text_z = float(self.get_parameter("text_z").value)

        self.busy = False
        self.busy_lock = threading.Lock()
        self.current_goal_handle = None

        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        command_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, marker_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, command_qos)
        self.result_flag_pub = self.create_publisher(Int8, self.result_flag_topic, command_qos)
        self.command_sub = self.create_subscription(
            Int32, self.command_topic, self.command_callback, command_qos)
        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_to_pose_action)

        self.get_logger().info(f"Listening for task ids on {self.command_topic}")
        self.get_logger().info(f"Publishing task status on {self.status_topic}")
        self.get_logger().info(f"Publishing task result flag on {self.result_flag_topic}")
        for task_id, task_file in sorted(self.task_map.items()):
            self.get_logger().info(f"  {task_id} -> {task_file}")

    def parse_task_map(self, entries):
        task_map = {}
        for entry in entries:
            text = str(entry)
            if ":" not in text:
                self.get_logger().warn(f"Ignoring invalid task_map entry: {text}")
                continue
            key_text, path_text = text.split(":", 1)
            try:
                task_id = int(key_text.strip())
            except ValueError:
                self.get_logger().warn(f"Ignoring invalid task id: {key_text}")
                continue

            path = os.path.expanduser(path_text.strip())
            if not os.path.isabs(path):
                path = os.path.join(self.task_dir, path)
            task_map[task_id] = path
        return task_map

    def command_callback(self, msg):
        task_id = int(msg.data)
        task_file = self.task_map.get(task_id)
        if task_file is None:
            self.get_logger().warn(f"No task mapped for id {task_id}")
            self.publish_status(task_id, "unknown_task")
            self.publish_result_flag(False)
            return

        with self.busy_lock:
            if self.busy:
                self.get_logger().warn(
                    f"Task already running; ignoring id {task_id}")
                self.publish_status(task_id, "busy")
                self.publish_result_flag(False)
                return
            self.busy = True

        thread = threading.Thread(
            target=self.run_task_thread, args=(task_id, task_file), daemon=True)
        thread.start()

    def run_task_thread(self, task_id, task_file):
        try:
            self.get_logger().info(f"Starting task id {task_id}: {task_file}")
            waypoints, poses = self.load_task(task_file)
            self.publish_markers(waypoints, active_index=0)
            self.publish_status(
                task_id, "task_started", total=len(poses), task_file=task_file)

            if not self.nav_client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError(
                    f"Action server '{self.navigate_to_pose_action}' is not available")

            for index, pose in enumerate(poses):
                self.publish_markers(waypoints, active_index=index, completed_count=index)
                name = waypoints[index].get("name", f"waypoint_{index + 1}")
                self.get_logger().info(
                    f"Task {task_id}: sending waypoint {index + 1}/{len(poses)}: {name}")
                self.publish_status(
                    task_id, "waypoint_started", index=index + 1,
                    total=len(poses), waypoint=name)

                goal = NavigateToPose.Goal()
                goal.pose = pose
                status = self.send_goal_and_wait(goal)
                if status != 4:
                    self.publish_markers(waypoints, active_index=index, failed_index=index)
                    self.get_logger().error(
                        f"Task {task_id}: waypoint {name} failed, status={status}")
                    self.publish_status(
                        task_id, "waypoint_failed", index=index + 1,
                        total=len(poses), waypoint=name, status=status)
                    self.publish_status(task_id, "task_failed", status=status)
                    self.publish_result_flag(False)
                    return

                self.publish_markers(
                    waypoints, active_index=index + 1, completed_count=index + 1)
                self.get_logger().info(
                    f"Task {task_id}: reached waypoint {index + 1}/{len(poses)}: {name}")
                self.publish_status(
                    task_id, "waypoint_succeeded", index=index + 1,
                    total=len(poses), waypoint=name, status=status)

            self.publish_markers(waypoints, active_index=-1, completed_count=len(poses))
            self.get_logger().info(f"Task {task_id} finished")
            self.publish_status(task_id, "task_succeeded", total=len(poses), status=4)
            self.publish_result_flag(True)
        except Exception as exc:
            self.get_logger().error(f"Task {task_id} failed: {exc}")
            self.publish_status(task_id, "task_failed", error=str(exc))
            self.publish_result_flag(False)
        finally:
            with self.busy_lock:
                self.busy = False
                self.current_goal_handle = None

    def publish_status(
        self,
        task_id,
        event,
        index=None,
        total=None,
        waypoint=None,
        status=None,
        task_file=None,
        error=None):
        fields = [
            f"task_id:{task_id}",
            f"event:{event}",
        ]
        if index is not None:
            fields.append(f"index:{index}")
        if total is not None:
            fields.append(f"total:{total}")
        if waypoint is not None:
            fields.append(f"waypoint:{waypoint}")
        if status is not None:
            fields.append(f"status:{status}")
        if task_file is not None:
            fields.append(f"task_file:{task_file}")
        if error is not None:
            fields.append(f"error:{error}")

        msg = String()
        msg.data = " ".join(fields)
        self.status_pub.publish(msg)

    def publish_result_flag(self, success):
        msg = Int8()
        msg.data = 1 if success else 0
        self.result_flag_pub.publish(msg)

    def load_task(self, task_file):
        with open(task_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        waypoints = data.get("waypoints", []) if isinstance(data, dict) else []
        if not waypoints:
            raise RuntimeError(f"No waypoints in {task_file}")

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

    def send_goal_and_wait(self, goal):
        send_event = threading.Event()
        send_result = {}

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future: self.set_future_result(future, send_result, send_event))
        send_event.wait()

        goal_handle = send_result.get("result")
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Waypoint goal was rejected")
        self.current_goal_handle = goal_handle

        result_event = threading.Event()
        action_result = {}
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.set_future_result(future, action_result, result_event))
        result_event.wait()

        result = action_result.get("result")
        return result.status if result is not None else None

    def set_future_result(self, future, store, event):
        try:
            store["result"] = future.result()
        except Exception as exc:
            store["exception"] = exc
        finally:
            event.set()

    def publish_markers(self, waypoints, active_index=-1, completed_count=0, failed_index=-1):
        markers = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        stamp = self.get_clock().now().to_msg()
        for index, waypoint in enumerate(waypoints):
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
        node = WaypointTaskDispatcher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"waypoint_task_dispatcher failed: {exc}")
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
