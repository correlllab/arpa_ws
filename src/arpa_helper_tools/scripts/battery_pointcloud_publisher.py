#!/usr/bin/env python3
"""
Battery Point Cloud Publisher (simulation)

Loads the real accumulated_pointcloud.ply captured from the RealSense and
publishes it on /realsense/accumulated_point_cloud so the existing RViz
display picks it up.

Services:
  /toggle_battery_pointcloud  (std_srvs/Trigger)  — show / hide toggle
"""

import os
import struct
import threading
from typing import Any

import numpy as np
import rclpy

try:
    import open3d as o3d
except Exception as e:
    o3d = None  # type: Any
    _O3D_IMPORT_ERROR = e
else:
    _O3D_IMPORT_ERROR = None
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from sensor_msgs_py import point_cloud2

_FRAME_ID = "floor_link"
_TOPIC = "/realsense/accumulated_point_cloud"

def _find_ply() -> str | None:
    from ament_index_python.packages import get_package_share_directory
    candidates = []
    try:
        candidates.append(os.path.join(
            get_package_share_directory("arpa_helper_tools"),
            "resources", "accumulated_pointcloud.ply"
        ))
    except Exception:
        pass
    try:
        candidates.append(os.path.join(
            get_package_share_directory("cl_realsense"),
            "scripts", "accumulated_pointclouds", "accumulated_pointcloud.ply"
        ))
    except Exception:
        pass
    # Fallback: source-tree relative path (dev-mode)
    candidates.append(os.path.join(
        os.path.dirname(__file__), "..", "resources", "accumulated_pointcloud.ply"
    ))
    for p in candidates:
        resolved = os.path.realpath(p)
        if os.path.isfile(resolved):
            return resolved
    return None


def _pcd_to_msg(pcd: Any, stamp) -> PointCloud2:
    points = np.asarray(pcd.points, dtype=np.float32)
    header = Header()
    header.stamp = stamp
    header.frame_id = _FRAME_ID

    if len(points) == 0:
        return point_cloud2.create_cloud_xyz32(header, np.zeros((0, 3), dtype=np.float32))

    colors = np.asarray(pcd.colors)
    if colors.size > 0:
        colors_uint8 = (colors * 255).astype(np.uint8)
        rgb_uint32 = (
            np.left_shift(colors_uint8[:, 0].astype(np.uint32), 16)
            | np.left_shift(colors_uint8[:, 1].astype(np.uint32), 8)
            | colors_uint8[:, 2].astype(np.uint32)
        )
        rgb_float32 = rgb_uint32.view(np.float32)
    else:
        grey = struct.unpack("f", struct.pack("I", (120 << 16) | (130 << 8) | 135))[0]
        rgb_float32 = np.full(len(points), grey, dtype=np.float32)

    cloud_data = np.hstack([points, rgb_float32[:, np.newaxis]])
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    return point_cloud2.create_cloud(header, fields, cloud_data)


class BatteryPointCloudPublisher(Node):
    def __init__(self):
        super().__init__("battery_pointcloud_publisher")

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(PointCloud2, _TOPIC, latched_qos)
        self._toggle_srv = self.create_service(
            Trigger, "/toggle_battery_pointcloud", self._handle_toggle
        )

        self._cloud_msg = None
        self._visible = False

        if o3d is None:
            self.get_logger().error(
                "open3d failed to import (often NumPy/scipy mismatch). "
                "Battery point cloud disabled. Try: pip install 'numpy<2'. "
                f"Import error: {_O3D_IMPORT_ERROR}"
            )
            return

        ply_path = _find_ply()
        if ply_path is None:
            self.get_logger().error(
                "accumulated_pointcloud.ply not found. "
                "Place it in cl_realsense/scripts/accumulated_pointclouds/"
            )
            return

        pcd = o3d.io.read_point_cloud(ply_path)
        if not pcd.has_points():
            self.get_logger().error(f"PLY is empty: {ply_path}")
            return

        self._cloud_msg = _pcd_to_msg(pcd, self.get_clock().now().to_msg())
        self._pub.publish(self._cloud_msg)
        self._visible = True
        self.get_logger().info(
            f"Published real battery point cloud ({len(pcd.points)} pts) from {ply_path}"
        )

    def _build_empty(self) -> PointCloud2:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = _FRAME_ID
        return point_cloud2.create_cloud_xyz32(header, np.zeros((0, 3), dtype=np.float32))

    def _handle_toggle(self, _request, response):
        if self._cloud_msg is None:
            response.success = False
            response.message = "No point cloud loaded"
            return response

        if self._visible:
            self._pub.publish(self._build_empty())
            self._visible = False
            response.success = True
            response.message = "hidden"
            self.get_logger().info("Battery point cloud hidden")
        else:
            self._cloud_msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(self._cloud_msg)
            self._visible = True
            response.success = True
            response.message = "shown"
            self.get_logger().info("Battery point cloud shown")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BatteryPointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
