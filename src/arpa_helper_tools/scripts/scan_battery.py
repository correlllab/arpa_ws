#!/usr/bin/env python3
"""
Battery Scanning Helper Script

Iterates through predefined scanning poses in a serpentine pattern.
For each pose: plans the motion, waits for user approval, then executes.
Uses CoreNode from move_to_pose.py for planning and execution.
"""

import argparse
import random
import sys
import time

import rclpy
from core_functionality_node import CoreNode
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import time
import random

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np
from scipy.spatial.transform import Rotation as R


# 32 predefined scanning poses for battery inspection
# Reference: Back Center at x=1.0225, y=0.007, z=1.253
# Battery: 1.9m long (X axis), 1.5m wide (Y axis)
# Grid: 8 rows (along length) x 4 columns (along width)
# Row spacing: 1.9m / 7 = 0.271m
# Column spacing: 1.5m / 3 = 0.5m

"""
LOWER LEFT
root@localhost:~/ros2_ws# ros2 run tf2_ros tf2_echo floor_link tool0
[INFO] [1770251984.865388082] [tf2_echo]: Waiting for transform floor_link ->  tool0: Invalid frame ID "floor_link" passed to canTransform argument target_frame - frame does not exist
At time 1770251985.817181793
- Translation: [1.109, -0.743, 1.253]
- Rotation: in Quaternion (xyzw) [0.707, 0.707, 0.000, -0.000]
- Rotation: in RPY (radian) [-3.141, -0.000, 1.571]
- Rotation: in RPY (degree) [-179.995, -0.010, 89.995]
- Matrix:
  0.000  1.000 -0.000  1.109
  1.000 -0.000  0.000 -0.743
  0.000 -0.000 -1.000  1.253
  0.000  0.000  0.000  1.000



UPPER RIGHT
root@localhost:~/ros2_ws# ros2 run tf2_ros tf2_echo floor_link tool0
[INFO] [1770252099.275512291] [tf2_echo]: Waiting for transform floor_link ->  tool0: Invalid frame ID "floor_link" passed to canTransform argument target_frame - frame does not exist
At time 1770252100.237171449
- Translation: [-0.873, 0.631, 1.253]
- Rotation: in Quaternion (xyzw) [0.707, 0.707, 0.000, 0.000]
- Rotation: in RPY (radian) [3.142, 0.000, 1.571]
- Rotation: in RPY (degree) [179.998, 0.001, 90.002]
- Matrix:
 -0.000  1.000  0.000 -0.873
  1.000  0.000 -0.000  0.631
 -0.000  0.000 -1.000  1.253
  0.000  0.000  0.000  1.000
"""
SAVE_IMAGES = False

# Scan area bounds (X, Y)
LOWER_LEFT = [1.109, -0.743]   # Starting corner
UPPER_RIGHT = [-0.873, 0.631]  # Opposite corner
_Z_HEIGHT = 1.253

# Grid configuration
N_X_STEPS = 8 # Number of positions along X
N_Y_STEPS = 8  # Number of positions along Y

# When True, only visit the three outermost rows and columns (border band of depth 3)
only_outside_points = False

# Orientation quaternion (pointing down for scanning)
# Axis-aligned: RPY (180°, 0°, 90°) - tool pointing down (-Z), Y-axis forward
_QX, _QY, _QZ, _QW = 0.7071068, 0.7071068, 0.0, 0.0

# 9 scan orientations per spatial point:
#   straight down + ±45° pitch (tilt around world Y) x ±45° roll (tilt around world X)
#   Ordered as a 3×3 grid: pitch in {-45, 0, +45} × roll in {-45, 0, +45}
_SCAN_TILT_DEG = 0
_BASE_ROT = R.from_euler('xyz', [180.0, 0.0, 90.0], degrees=True)
_SCAN_ORIENTATIONS = []
# Straight down always first, then the 8 tilted orientations
_tilt_pairs = [(0.0, 0.0)] + [
    (_p, _r)
    for _p in [-_SCAN_TILT_DEG, 0.0, _SCAN_TILT_DEG]
    for _r in [-_SCAN_TILT_DEG, 0.0, _SCAN_TILT_DEG]
    if not (_p == 0.0 and _r == 0.0)
]
for _pitch, _roll in _tilt_pairs:
    _tilt = R.from_euler('y', _pitch, degrees=True) * R.from_euler('x', _roll, degrees=True)
    _q = (_tilt * _BASE_ROT).as_quat()  # [qx, qy, qz, qw]
    _SCAN_ORIENTATIONS.append((float(_q[0]), float(_q[1]), float(_q[2]), float(_q[3])))

# Generate X and Y positions from bounds
_X_POSITIONS = [LOWER_LEFT[0] + i * (UPPER_RIGHT[0] - LOWER_LEFT[0]) / (N_X_STEPS - 1) for i in range(N_X_STEPS)]
_Y_POSITIONS = [LOWER_LEFT[1] + i * (UPPER_RIGHT[1] - LOWER_LEFT[1]) / (N_Y_STEPS - 1) for i in range(N_Y_STEPS)]

# Generate poses in zigzag pattern (scan along Y at each X row, alternating Y direction)
_OUTSIDE_DEPTH = 2  # Number of outermost rows/columns to include when only_outside_points is True
scan_points = []
for row_idx, x_pos in enumerate(_X_POSITIONS):
    y_range = _Y_POSITIONS# if row_idx % 2 == 0 else list(reversed(_Y_POSITIONS))
    for col_idx, y_pos in enumerate(y_range):
        if only_outside_points:
            row_is_outside = row_idx < _OUTSIDE_DEPTH or row_idx >= N_X_STEPS - _OUTSIDE_DEPTH
            col_is_outside = col_idx < _OUTSIDE_DEPTH or col_idx >= N_Y_STEPS - _OUTSIDE_DEPTH
            if not (row_is_outside or col_is_outside):
                continue
        scan_points.append((x_pos, y_pos))


VARIABLE_ORIENTATION = False
def scan_points_to_pose_stamped(points, frame_id):
    pose_stamped_list = []
    for x, y in points:
        # Build orientation: tool Z down, tool Y toward origin in XY plane
        qx,qy,qz,qw = _QX, _QY, _QZ, _QW
        is_edge =(x in _X_POSITIONS[:_OUTSIDE_DEPTH]) or (x in _X_POSITIONS[-_OUTSIDE_DEPTH:]) or (y in _Y_POSITIONS[:_OUTSIDE_DEPTH]) or (y in _Y_POSITIONS[-_OUTSIDE_DEPTH:])
        if VARIABLE_ORIENTATION# or is_edge:
            z_hat = np.array([0.0, 0.0, -1.0])
            toward_origin = np.array([-x, -y, 0.0])
            norm = np.linalg.norm(toward_origin)
            y_hat = toward_origin / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0])
            x_hat = np.cross(y_hat, z_hat)
            rot = np.column_stack([x_hat, y_hat, z_hat])
            qx, qy, qz, qw = R.from_matrix(rot).as_quat()

        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = _Z_HEIGHT
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        pose_stamped_list.append(ps)
    return pose_stamped_list
    


FRAME_ID = "floor_link"
# Collision plane configuration
_PLANE_ID = "battery_do_not_cross"
_PLANE_Z = 0.9  # Z height of the plane (below scan height)
_PLANE_THICKNESS = 0.02  # Thin plane
# Compute plane size and center from scan bounds
_PLANE_SIZE_X = abs(UPPER_RIGHT[0] - LOWER_LEFT[0]) + 0.2  # Add margin
_PLANE_SIZE_Y = abs(UPPER_RIGHT[1] - LOWER_LEFT[1]) + 0.2
_PLANE_CENTER_X = (LOWER_LEFT[0] + UPPER_RIGHT[0]) / 2.0
_PLANE_CENTER_Y = (LOWER_LEFT[1] + UPPER_RIGHT[1]) / 2.0




def build_scan_marker_array(node, pose_array):
    """Build a MarkerArray with an arrow marker for each scan pose (PoseStamped list)."""
    marker_array = MarkerArray()
    for i, pose in enumerate(pose_array):
        m = Marker()
        m.header.frame_id = pose.header.frame_id
        m.header.stamp = node.get_clock().now().to_msg()
        m.ns = "scan_poses"
        m.id = i
        m.type = Marker.ARROW
        m.action = Marker.ADD

        m.pose = pose.pose

        m.scale.x = 0.15  # arrow length
        m.scale.y = 0.02  # arrow shaft diameter
        m.scale.z = 0.02  # arrow head diameter

        # Pending poses are blue
        m.color = ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.8)

        marker_array.markers.append(m)
    return marker_array


def update_marker_color(marker_array, index, r, g, b, a=1.0):
    """Update a single marker's color in the array."""
    marker_array.markers[index].color = ColorRGBA(r=r, g=g, b=b, a=a)


def main(args=None):
    parser = argparse.ArgumentParser(description="Battery scan poses; use --benchmark for run_benchmark.py.")
    parser.add_argument("--benchmark", action="store_true", help="Print BENCHMARK|... line to stdout for run_benchmark.py")
    parsed, unknown = parser.parse_known_args(args)
    benchmark_mode = parsed.benchmark

    rclpy.init(args=args)
    node = CoreNode()

    capture_client = node.create_client(Trigger, 'record_images/capture')

    # Clear any existing detections before the scan begins
    clear_det_client = node.create_client(Trigger, '/arpa_vision_node/clear_detections')
    if clear_det_client.wait_for_service(timeout_sec=5.0):
        future = clear_det_client.call_async(Trigger.Request())
        while not future.done():
            time.sleep(0.05)
        node.get_logger().info("Detections cleared before scan.")
    else:
        node.get_logger().warn("clear_detections service not available, skipping clear.")

    clear_acc_client = node.create_client(Trigger, 'pointcloud_accumulator/clear_arm_pointcloud')
    if clear_acc_client.wait_for_service(timeout_sec=5.0):
        future = clear_acc_client.call_async(Trigger.Request())
        while not future.done():
            time.sleep(0.05)
        node.get_logger().info("Arm pointcloud cleared before scan.")
    else:
        node.get_logger().warn("clear_arm_pointcloud service not available, skipping clear.")

    marker_pub = node.create_publisher(MarkerArray, '/scan_poses_markers', 10)
    pose_stamped_list = scan_points_to_pose_stamped(scan_points, FRAME_ID)
    pose_arr = node.get_tsp_order(pose_stamped_list)

    marker_array = build_scan_marker_array(node, pose_arr)
    node.trigger_behavior("ros2control")

    # Give publishers time to connect
    time.sleep(1.0)

    # Publish markers so they appear in rviz
    marker_pub.publish(marker_array)

    # Add collision plane to prevent planner from going below battery
    node.add_collision_plane(_PLANE_ID, FRAME_ID, _PLANE_CENTER_X, _PLANE_CENTER_Y, _PLANE_Z, _PLANE_SIZE_X, _PLANE_SIZE_Y, _PLANE_THICKNESS)

    # Publish markers again to ensure visibility
    marker_pub.publish(marker_array)


    node.get_logger().info(f"Battery scan: {len(pose_arr)} poses to visit")

    try:
        skipped_indices = []  # Track skipped poses for retry
        completed_indices = []  # Track successful poses
        plan_times_list = []  # Per-pose plan time (s) for --benchmark
        successes_list = []    # Per-pose 1/0 for --benchmark
        scan_start_time = time.time() if benchmark_mode else None

        # Use 9 tilted orientations for the outer-edge scan, straight-down only otherwise
        active_orientations = _SCAN_ORIENTATIONS# if only_outside_points else [(_QX, _QY, _QZ, _QW)]

        # First pass: visit all spatial positions, skip failures
        for i, pose in enumerate(pose_arr):
            # Highlight current position in yellow
            update_marker_color(marker_array, i, r=1.0, g=1.0, b=0.0)
            marker_pub.publish(marker_array)

            p = pose.pose.position
            node.get_logger().info(
                f"\n--- [{i+1}/{len(pose_arr)}] Position {i+1} ---"
                f"\n    x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

            orientation_successes = 0
            for j, (qx, qy, qz, qw) in enumerate([(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)]): #enumerate(active_orientations):
                if len(active_orientations) > 1:
                    node.get_logger().info(f"  Orientation [{j+1}/{len(active_orientations)}]")

                t0 = time.time()
                success = node.plan_to_pose(
                    p.x, p.y, p.z, qx, qy, qz, qw,
                    frame_id=pose.header.frame_id)
                plan_time_s = time.time() - t0

                if benchmark_mode:
                    plan_times_list.append(plan_time_s)

                if not success:

                    if benchmark_mode:
                        successes_list.append(0)
                    node.get_logger().warn(f"  Orientation {j+1} planning failed, skipping")
                    time.sleep(0.5)  # Brief pause before next attempt
                    continue

                if not node.execute_plan():
                    if benchmark_mode:
                        successes_list.append(0)
                    node.get_logger().error(f"  Orientation {j+1} execution failed, skipping")
                    time.sleep(0.5)
                    continue

                if benchmark_mode:
                    successes_list.append(1)
                orientation_successes += 1

                time.sleep(0.67)  # Brief pause to stabilize before capture

                if capture_client.service_is_ready() and SAVE_IMAGES:
                    future = capture_client.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                    if future.done():
                        node.get_logger().info(f"  Captured at orientation {j+1}: {future.result().message}")
                    else:
                        node.get_logger().warn(f"  Capture timed out at orientation {j+1}")
                else:
                    node.get_logger().warn(f"  Capture service not available at orientation {j+1}, skipping")

            if orientation_successes == 0:
                skipped_indices.append(i)
                update_marker_color(marker_array, i, r=1.0, g=0.0, b=0.0, a=0.8)
                node.get_logger().warn(f"All orientations failed at Position {i+1}")
            else:
                completed_indices.append(i)
                update_marker_color(marker_array, i, r=0.0, g=1.0, b=0.0)
                node.get_logger().info(
                    f"Completed Position {i+1} ({orientation_successes}/{len(active_orientations)} orientations)")
            marker_pub.publish(marker_array)

        node.get_logger().info("Battery scan complete.")

        if benchmark_mode and scan_start_time is not None:
            wall_s = time.time() - scan_start_time
            completed = len(completed_indices)
            skipped = len(skipped_indices)
            total = len(pose_arr)
            plan_times_csv = ",".join(f"{t:.4f}" for t in plan_times_list)
            successes_csv = ",".join(str(s) for s in successes_list)
            # Single line for run_benchmark.py to parse (must match parse_benchmark_line)
            print(f"BENCHMARK|{wall_s:.4f}|{completed}|{skipped}|{total}|{plan_times_csv}|{successes_csv}", flush=True)

    except KeyboardInterrupt:
        node.get_logger().info("Scan interrupted by user.")
    finally:
        node.remove_collision_plane(_PLANE_ID, FRAME_ID)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
