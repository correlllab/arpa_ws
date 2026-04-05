#!/usr/bin/env python3
"""
Parts Visualizer Node

Advertises /toggle_parts_markers (std_srvs/Trigger).  Each call toggles
between showing and hiding the Hyundai Ioniq parts list on
/parts_list_markers (sphere per part + text label).

Also keeps the legacy /show_parts_markers service that always forces show.
"""

import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory


_FRAME_ID = "floor_link"

_TYPE_COLORS = {
    "Nut":        ColorRGBA(r=0.2, g=0.9, b=0.2, a=0.9),
    "Screw":      ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.9),
    "Bolt":       ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9),
    "Screw Hole": ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.9),
}
_DEFAULT_COLOR = ColorRGBA(r=0.9, g=0.9, b=0.2, a=0.9)
_TEXT_COLOR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def _part_type(name: str) -> str:
    if name.startswith("Screw Hole"):
        return "Screw Hole"
    if name.startswith("Screw"):
        return "Screw"
    if name.startswith("Nut"):
        return "Nut"
    if name.startswith("Bolt"):
        return "Bolt"
    return ""


class PartsVisualizerNode(Node):
    def __init__(self):
        super().__init__("parts_visualizer_node")

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "/parts_list_markers", latched_qos
        )

        self._toggle_srv = self.create_service(
            Trigger, "/toggle_parts_markers", self._handle_toggle
        )
        self._show_srv = self.create_service(
            Trigger, "/show_parts_markers", self._handle_show
        )

        self._parts = self._load_parts()
        self._visible = False
        self.get_logger().info(
            f"Parts visualizer ready — {len(self._parts)} parts loaded. "
            "Call /toggle_parts_markers or /show_parts_markers."
        )

    def _load_parts(self) -> dict:
        try:
            pkg_share = get_package_share_directory("arpa_helper_tools")
        except Exception:
            pkg_share = os.path.join(
                os.path.dirname(__file__), "..", "resources"
            )
        json_path = os.path.join(
            pkg_share, "resources", "parts_lists", "hyundai_ioniq_parts_list.json"
        )
        if not os.path.isfile(json_path):
            self.get_logger().error(f"Parts list not found at {json_path}")
            return {}
        with open(json_path) as f:
            return json.load(f)

    def _build_markers(self) -> MarkerArray:
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()
        marker_id = 0

        for name, (x, y, z) in self._parts.items():
            ptype = _part_type(name)
            color = _TYPE_COLORS.get(ptype, _DEFAULT_COLOR)

            sphere = Marker()
            sphere.header.frame_id = _FRAME_ID
            sphere.header.stamp = now
            sphere.ns = "parts_spheres"
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(x)
            sphere.pose.position.y = float(y)
            sphere.pose.position.z = float(z)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.015
            sphere.scale.y = 0.015
            sphere.scale.z = 0.015
            sphere.color = color
            ma.markers.append(sphere)
            marker_id += 1

            text = Marker()
            text.header.frame_id = _FRAME_ID
            text.header.stamp = now
            text.ns = "parts_labels"
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(x)
            text.pose.position.y = float(y)
            text.pose.position.z = float(z) + 0.04
            text.pose.orientation.w = 1.0
            text.scale.z = 0.025
            text.color = _TEXT_COLOR
            text.text = name
            ma.markers.append(text)
            marker_id += 1

        return ma

    def _build_delete_all(self) -> MarkerArray:
        ma = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        return ma

    def _publish_show(self):
        ma = self._build_markers()
        self._marker_pub.publish(ma)
        self._visible = True
        self.get_logger().info(
            f"Published {len(self._parts)} part markers on /parts_list_markers"
        )

    def _publish_hide(self):
        self._marker_pub.publish(self._build_delete_all())
        self._visible = False
        self.get_logger().info("Cleared part markers")

    def _handle_toggle(self, _request, response):
        if not self._parts:
            response.success = False
            response.message = "No parts loaded"
            return response

        if self._visible:
            self._publish_hide()
            response.success = True
            response.message = "hidden"
        else:
            self._publish_show()
            response.success = True
            response.message = f"shown {len(self._parts)} parts"
        return response

    def _handle_show(self, _request, response):
        if not self._parts:
            response.success = False
            response.message = "No parts loaded"
            return response
        self._publish_show()
        response.success = True
        response.message = f"Published {len(self._parts)} parts"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PartsVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
