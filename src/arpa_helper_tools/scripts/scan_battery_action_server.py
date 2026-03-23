#!/usr/bin/env python3
"""
Battery scan ROS2 action server.
Action: custom_ros_messages/action/ScanBattery
Feedback: points_explored (int32), total_points (int32)
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from core_functionality_node import CoreNode
from custom_ros_messages.action import ScanBattery


# ---------------------------------------------------------------------------
# Scan grid configuration (mirrors scan_battery.py)
# ---------------------------------------------------------------------------
LOWER_LEFT  = [1.0, -0.75]
UPPER_RIGHT = [-0.90, 0.65]
_Z_HEIGHT   = 1.25
N_X_STEPS   = 8
N_Y_STEPS   = 8
only_outside_points = False
_OUTSIDE_DEPTH = 2

_QX, _QY, _QZ, _QW = 0.7071068, 0.7071068, 0.0, 0.0
_BASE_ROT = R.from_euler('xyz', [180.0, 0.0, 90.0], degrees=True)
_SCAN_TILT_DEG = 0
_tilt_pairs = [(0.0, 0.0)] + [
    (_p, _r)
    for _p in [-_SCAN_TILT_DEG, 0.0, _SCAN_TILT_DEG]
    for _r in [-_SCAN_TILT_DEG, 0.0, _SCAN_TILT_DEG]
    if not (_p == 0.0 and _r == 0.0)
]
_SCAN_ORIENTATIONS = []
for _pitch, _roll in _tilt_pairs:
    _tilt = R.from_euler('y', _pitch, degrees=True) * R.from_euler('x', _roll, degrees=True)
    _q = (_tilt * _BASE_ROT).as_quat()
    _SCAN_ORIENTATIONS.append((float(_q[0]), float(_q[1]), float(_q[2]), float(_q[3])))

_X_POSITIONS = [LOWER_LEFT[0] + i * (UPPER_RIGHT[0] - LOWER_LEFT[0]) / (N_X_STEPS - 1) for i in range(N_X_STEPS)]
_Y_POSITIONS = [LOWER_LEFT[1] + i * (UPPER_RIGHT[1] - LOWER_LEFT[1]) / (N_Y_STEPS - 1) for i in range(N_Y_STEPS)]

scan_points = []
for row_idx, x_pos in enumerate(_X_POSITIONS):
    for col_idx, y_pos in enumerate(_Y_POSITIONS):
        if only_outside_points:
            row_is_outside = row_idx < _OUTSIDE_DEPTH or row_idx >= N_X_STEPS - _OUTSIDE_DEPTH
            col_is_outside = col_idx < _OUTSIDE_DEPTH or col_idx >= N_Y_STEPS - _OUTSIDE_DEPTH
            if not (row_is_outside or col_is_outside):
                continue
        scan_points.append((x_pos, y_pos))

FRAME_ID = "floor_link"


def _scan_points_to_pose_stamped(points):
    poses = []
    for x, y in points:
        ps = PoseStamped()
        ps.header.frame_id = FRAME_ID
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = _Z_HEIGHT
        ps.pose.orientation.x = _QX
        ps.pose.orientation.y = _QY
        ps.pose.orientation.z = _QZ
        ps.pose.orientation.w = _QW
        poses.append(ps)
    return poses


def _build_marker_array(node, pose_array):
    ma = MarkerArray()
    for i, pose in enumerate(pose_array):
        m = Marker()
        m.header.frame_id = pose.header.frame_id
        m.header.stamp = node.get_clock().now().to_msg()
        m.ns = "scan_poses"
        m.id = i
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x = 0.15
        m.scale.y = 0.02
        m.scale.z = 0.02
        m.color = ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.8)
        ma.markers.append(m)
    return ma


def _set_marker_color(marker_array, index, r, g, b, a=1.0):
    marker_array.markers[index].color = ColorRGBA(r=r, g=g, b=b, a=a)


# ---------------------------------------------------------------------------
# Action server node
# ---------------------------------------------------------------------------
class ScanBatteryServer(Node):
    def __init__(self):
        super().__init__('scan_battery_action_server')
        self._core = CoreNode()
        self._action_server = ActionServer(
            self,
            ScanBattery,
            'scan_battery',
            execute_callback=self._execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda cancel: CancelResponse.ACCEPT,
        )
        self.get_logger().info("ScanBattery action server ready")

    # ------------------------------------------------------------------
    def _execute(self, goal_handle):
        save_images = goal_handle.request.save_images
        self.get_logger().info(f"Starting battery scan (save_images={save_images})")

        node = self._core
        feedback_msg = ScanBattery.Feedback()
        result = ScanBattery.Result()

        # --- optional service calls: clear detections & pointcloud ---
        for srv_name in ('arpa_vision_node/clear_detections',
                         'pointcloud_accumulator/clear_arm_pointcloud'):
            client = node.create_client(Trigger, srv_name)
            if client.wait_for_service(timeout_sec=3.0):
                future = client.call_async(Trigger.Request())
                deadline = time.time() + 3.0
                while not future.done() and time.time() < deadline:
                    time.sleep(0.05)
                node.get_logger().info(f"Cleared: {srv_name}")
            else:
                node.get_logger().warn(f"Service not available: {srv_name}")

        # --- capture client ---
        capture_client = node.create_client(Trigger, 'record_images/capture')

        # --- build pose list ---
        marker_pub = node.create_publisher(MarkerArray, '/scan_poses_markers', 10)
        pose_stamped_list = _scan_points_to_pose_stamped(scan_points)
        pose_arr = node.get_tsp_order(pose_stamped_list)

        # Append edge poses along the leftmost X column (mirrors scan_battery.py)
        for y in _Y_POSITIONS:
            x = _X_POSITIONS[0]
            z_hat = np.array([0.0, 0.0, -1.0])
            y_hat = np.array([-1.0, 0.0, 0.0])
            x_hat = np.cross(y_hat, z_hat)
            rot = np.column_stack([x_hat, y_hat, z_hat])
            qx, qy, qz, qw = R.from_matrix(rot).as_quat()
            ps = PoseStamped()
            ps.header.frame_id = FRAME_ID
            ps.pose.position.x = x - 0.05
            ps.pose.position.y = y
            ps.pose.position.z = _Z_HEIGHT
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            pose_arr.append(ps)

        total = len(pose_arr)
        marker_array = _build_marker_array(node, pose_arr)

        node.trigger_behavior("ros2control")
        time.sleep(1.0)
        marker_pub.publish(marker_array)
        node.add_collision_plane()
        marker_pub.publish(marker_array)

        node.get_logger().info(f"Battery scan: {total} poses to visit")

        completed_indices = []
        skipped_indices = []

        try:
            for i, pose in enumerate(pose_arr):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.total_poses = total
                    result.completed = len(completed_indices)
                    result.skipped = len(skipped_indices)
                    return result

                _set_marker_color(marker_array, i, r=1.0, g=1.0, b=0.0)
                marker_pub.publish(marker_array)

                p = pose.pose.position
                node.get_logger().info(
                    f"\n--- [{i+1}/{total}] x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

                success = node.plan_to_pose(
                    p.x, p.y, p.z,
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                    frame_id=pose.header.frame_id)

                if not success or not node.execute_plan():
                    skipped_indices.append(i)
                    _set_marker_color(marker_array, i, r=1.0, g=0.0, b=0.0, a=0.8)
                    node.get_logger().warn(f"  Failed at pose {i+1}, skipping")
                    time.sleep(0.5)
                else:
                    completed_indices.append(i)
                    _set_marker_color(marker_array, i, r=0.0, g=1.0, b=0.0)
                    time.sleep(0.67)

                    if save_images and capture_client.service_is_ready():
                        future = capture_client.call_async(Trigger.Request())
                        deadline = time.time() + 2.0
                        while not future.done() and time.time() < deadline:
                            time.sleep(0.05)

                marker_pub.publish(marker_array)

                # Send feedback: how many points have been explored so far
                feedback_msg.points_explored = i + 1
                feedback_msg.total_points = total
                goal_handle.publish_feedback(feedback_msg)

        except Exception as e:
            node.get_logger().error(f"Scan error: {e}")
            goal_handle.abort()
            result.success = False
            result.total_poses = total
            result.completed = len(completed_indices)
            result.skipped = len(skipped_indices)
            return result

        node.get_logger().info(
            f"Scan complete: {len(completed_indices)}/{total} succeeded, "
            f"{len(skipped_indices)} skipped")

        goal_handle.succeed()
        result.success = True
        result.total_poses = total
        result.completed = len(completed_indices)
        result.skipped = len(skipped_indices)
        return result


def main(args=None):
    rclpy.init(args=args)
    server = ScanBatteryServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server._core.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
