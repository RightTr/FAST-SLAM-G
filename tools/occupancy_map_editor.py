#!/usr/bin/env python3

import copy
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

try:
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
except ImportError:
    get_package_prefix = None
    get_package_share_directory = None


VALUE_BY_MODE = {
    "free": 0,
    "clear": 0,
    "occupied": 100,
    "obstacle": 100,
    "unknown": -1,
}


def default_map_dir():
    if get_package_prefix is not None and get_package_share_directory is not None:
        try:
            package_prefix = get_package_prefix("fast_lio_sam_g")
            workspace_root = os.path.dirname(os.path.dirname(package_prefix))
            source_map_dir = os.path.join(workspace_root, "src", "FAST-SLAM-G", "map")
            if os.path.isdir(source_map_dir):
                return source_map_dir
            return os.path.join(get_package_share_directory("fast_lio_sam_g"), "map")
        except Exception:
            pass

    return os.path.abspath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "map"))


class OccupancyMapEditor(Node):
    def __init__(self):
        super().__init__("occupancy_map_editor")

        map_dir = default_map_dir()
        self.declare_parameter("input_map_topic", "/map_raw")
        self.declare_parameter("output_map_topic", "/map")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("edits_file", os.path.join(map_dir, "map_edits.yaml"))
        self.declare_parameter("mode", "free")
        self.declare_parameter("brush_radius", 0.20)
        self.declare_parameter("autosave", True)
        self.declare_parameter("export_service", "~/export_map")
        self.declare_parameter("export_map_yaml", os.path.join(map_dir, "edited_map.yaml"))
        self.declare_parameter("occupied_thresh", 0.65)
        self.declare_parameter("free_thresh", 0.25)

        input_map_topic = self.get_parameter("input_map_topic").value
        output_map_topic = self.get_parameter("output_map_topic").value
        clicked_point_topic = self.get_parameter("clicked_point_topic").value
        self.edits_file = os.path.expanduser(self.get_parameter("edits_file").value)
        self.autosave = bool(self.get_parameter("autosave").value)
        export_service = self.get_parameter("export_service").value

        self.edits = self.load_edits()
        self.latest_map = None
        self.edited_map = None

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        point_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, input_map_topic, self.map_callback, map_qos)
        self.point_sub = self.create_subscription(
            PointStamped, clicked_point_topic, self.clicked_point_callback, point_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, output_map_topic, map_qos)
        self.export_srv = self.create_service(Trigger, export_service, self.export_map_callback)
        self.add_on_set_parameters_callback(self.on_parameters)

        self.get_logger().info(
            f"Editing {input_map_topic} -> {output_map_topic}; "
            f"click topic {clicked_point_topic}; edits file {self.edits_file or '<disabled>'}")
        self.get_logger().info(
            "Set mode with: ros2 param set /occupancy_map_editor mode free|occupied|unknown")
        self.get_logger().info(
            "Export edited gridmap with: ros2 service call /occupancy_map_editor/export_map std_srvs/srv/Trigger {}")

    def on_parameters(self, params):
        for param in params:
            if param.name == "mode":
                mode = str(param.value).lower()
                if mode not in VALUE_BY_MODE:
                    return SetParametersResult(
                        successful=False,
                        reason="mode must be one of: free, occupied, unknown")
            if param.name == "brush_radius" and float(param.value) < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason="brush_radius must be >= 0")
            if param.name in ("occupied_thresh", "free_thresh"):
                value = float(param.value)
                if value < 0.0 or value > 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be in [0, 1]")
        return SetParametersResult(successful=True)

    def load_edits(self):
        if not self.edits_file or not os.path.exists(self.edits_file):
            return []

        with open(self.edits_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        edits = data.get("edits", []) if isinstance(data, dict) else []
        if not isinstance(edits, list):
            self.get_logger().warn("edits_file has invalid format; ignoring")
            return []
        return edits

    def save_edits(self):
        if not self.edits_file:
            return
        os.makedirs(os.path.dirname(self.edits_file), exist_ok=True)
        with open(self.edits_file, "w", encoding="utf-8") as f:
            yaml.safe_dump({"edits": self.edits}, f, sort_keys=False)

    def map_callback(self, msg):
        self.latest_map = msg
        self.publish_edited_map()

    def clicked_point_callback(self, msg):
        if self.latest_map is None:
            self.get_logger().warn("No map received yet; click ignored")
            return

        mode = str(self.get_parameter("mode").value).lower()
        radius = float(self.get_parameter("brush_radius").value)
        value = VALUE_BY_MODE[mode]

        edit = {
            "x": float(msg.point.x),
            "y": float(msg.point.y),
            "radius": radius,
            "value": value,
            "mode": mode,
            "stamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        self.edits.append(edit)
        if self.autosave:
            self.save_edits()

        self.publish_edited_map()
        self.get_logger().info(
            f"{mode} edit at ({edit['x']:.2f}, {edit['y']:.2f}), radius {radius:.2f} m")

    def publish_edited_map(self):
        if self.latest_map is None:
            return

        grid = copy.deepcopy(self.latest_map)
        for edit in self.edits:
            self.apply_edit(grid, edit)
        self.edited_map = grid
        self.map_pub.publish(grid)

    def apply_edit(self, grid, edit):
        try:
            x = float(edit["x"])
            y = float(edit["y"])
            radius = max(0.0, float(edit.get("radius", 0.0)))
            value = int(edit.get("value", VALUE_BY_MODE.get(str(edit.get("mode", "free")), 0)))
        except (TypeError, ValueError, KeyError):
            return

        info = grid.info
        res = float(info.resolution)
        if res <= 0.0 or info.width == 0 or info.height == 0:
            return

        center_x = int(math.floor((x - info.origin.position.x) / res))
        center_y = int(math.floor((y - info.origin.position.y) / res))
        radius_cells = max(0, int(math.ceil(radius / res)))
        radius_sq = radius * radius

        for cell_y in range(center_y - radius_cells, center_y + radius_cells + 1):
            if cell_y < 0 or cell_y >= info.height:
                continue
            for cell_x in range(center_x - radius_cells, center_x + radius_cells + 1):
                if cell_x < 0 or cell_x >= info.width:
                    continue
                world_x = info.origin.position.x + (cell_x + 0.5) * res
                world_y = info.origin.position.y + (cell_y + 0.5) * res
                if radius > 0.0 and (world_x - x) ** 2 + (world_y - y) ** 2 > radius_sq:
                    continue
                index = cell_y * info.width + cell_x
                grid.data[index] = value

    def export_map_callback(self, request, response):
        del request
        if self.edited_map is None:
            response.success = False
            response.message = "No edited map received yet"
            return response

        yaml_path = os.path.expanduser(self.get_parameter("export_map_yaml").value)
        if not yaml_path:
            response.success = False
            response.message = "export_map_yaml parameter is empty"
            return response

        try:
            pgm_path = self.write_map_files(self.edited_map, yaml_path)
        except Exception as exc:
            response.success = False
            response.message = f"Export failed: {exc}"
            return response

        response.success = True
        response.message = f"Exported edited map: {yaml_path}, {pgm_path}"
        self.get_logger().info(response.message)
        return response

    def write_map_files(self, grid, yaml_path):
        export_dir = os.path.dirname(yaml_path)
        if export_dir:
            os.makedirs(export_dir, exist_ok=True)
        base_path, _ = os.path.splitext(yaml_path)
        pgm_path = base_path + ".pgm"

        occupied_thresh = float(self.get_parameter("occupied_thresh").value)
        free_thresh = float(self.get_parameter("free_thresh").value)
        occupied_cutoff = int(round(occupied_thresh * 100.0))
        free_cutoff = int(round(free_thresh * 100.0))

        width = grid.info.width
        height = grid.info.height
        with open(pgm_path, "wb") as f:
            f.write(f"P5\n# CREATOR: occupancy_map_editor\n{width} {height}\n255\n".encode("ascii"))
            for y in range(height - 1, -1, -1):
                row = bytearray()
                for x in range(width):
                    value = int(grid.data[y * width + x])
                    if value < 0:
                        pixel = 205
                    elif value >= occupied_cutoff:
                        pixel = 0
                    elif value <= free_cutoff:
                        pixel = 254
                    else:
                        pixel = 205
                    row.append(pixel)
                f.write(row)

        origin = grid.info.origin
        yaw = self.yaw_from_quaternion(origin.orientation)
        map_yaml = {
            "image": os.path.basename(pgm_path),
            "resolution": float(grid.info.resolution),
            "origin": [
                float(origin.position.x),
                float(origin.position.y),
                float(yaw),
            ],
            "negate": 0,
            "occupied_thresh": occupied_thresh,
            "free_thresh": free_thresh,
        }
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(map_yaml, f, sort_keys=False)
        return pgm_path

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapEditor()
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
