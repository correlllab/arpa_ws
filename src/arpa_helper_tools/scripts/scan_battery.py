#!/usr/bin/env python3
"""
Battery Scanning Helper Script

Iterates through predefined scanning poses in a serpentine pattern.
For each pose: plans the motion, waits for user approval, then executes.
Uses CoreNode from move_to_pose.py for planning and execution.
"""

import rclpy
from core_functionality_node import CoreNode
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped
import time
import random

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


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

# Scan area bounds (X, Y)
LOWER_LEFT = [1.109, -0.743]   # Starting corner
UPPER_RIGHT = [-0.873, 0.631]  # Opposite corner
_Z_HEIGHT = 1.253

# Grid configuration
N_X_STEPS = 8 # Number of positions along X
N_Y_STEPS = 8  # Number of positions along Y

# Orientation quaternion (pointing down for scanning)
# Axis-aligned: RPY (180°, 0°, 90°) - tool pointing down (-Z), Y-axis forward
_QX, _QY, _QZ, _QW = 0.7071068, 0.7071068, 0.0, 0.0

# Generate X and Y positions from bounds
_X_POSITIONS = [LOWER_LEFT[0] + i * (UPPER_RIGHT[0] - LOWER_LEFT[0]) / (N_X_STEPS - 1) for i in range(N_X_STEPS)]
_Y_POSITIONS = [LOWER_LEFT[1] + i * (UPPER_RIGHT[1] - LOWER_LEFT[1]) / (N_Y_STEPS - 1) for i in range(N_Y_STEPS)]

# Generate poses in zigzag pattern (scan along Y at each X row, alternating Y direction)
scan_points = []
for row_idx, x_pos in enumerate(_X_POSITIONS):
    y_range = _Y_POSITIONS# if row_idx % 2 == 0 else list(reversed(_Y_POSITIONS))
    for col_idx, y_pos in enumerate(y_range):
        scan_points.append((x_pos, y_pos))

def scan_points_to_pose_stamped(points, frame_id):
    pose_stamped_list = []
    for x, y in points:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = _Z_HEIGHT
        ps.pose.orientation.x = _QX
        ps.pose.orientation.y = _QY
        ps.pose.orientation.z = _QZ
        ps.pose.orientation.w = _QW
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
    rclpy.init(args=args)
    node = CoreNode()

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

        # First pass: visit all poses, skip failures
        for i, pose in enumerate(pose_arr):
            # Highlight current pose in yellow
            update_marker_color(marker_array, i, r=1.0, g=1.0, b=0.0)
            marker_pub.publish(marker_array)

            p = pose.pose.position
            node.get_logger().info(
                f"\n--- [{i+1}/{len(pose_arr)}] Pose {i+1} ---"
                f"\n    x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

            o = pose.pose.orientation
            success = node.plan_to_pose(
                p.x, p.y, p.z, o.x, o.y, o.z, o.w,
                frame_id=pose.header.frame_id)

            if not success:
                skipped_indices.append(i)
                update_marker_color(marker_array, i, r=1.0, g=0.5, b=0.0, a=0.8)
                marker_pub.publish(marker_array)
                node.get_logger().warn(f"Skipping Pose {i+1} (will retry later)")
                continue

            if not node.execute_plan():
                skipped_indices.append(i)
                update_marker_color(marker_array, i, r=1.0, g=0.0, b=0.0, a=0.8)
                marker_pub.publish(marker_array)
                node.get_logger().error(f"Execution failed for Pose {i+1} (will retry later)")
                continue
            completed_indices.append(i)
            update_marker_color(marker_array, i, r=0.0, g=1.0, b=0.0)
            marker_pub.publish(marker_array)
            node.get_logger().info(f"Completed Pose {i+1}")

        # Retry passes: keep retrying from random successful waypoints
        # retry_round = 0
        # while skipped_indices and completed_indices:
        #     retry_round += 1
        #     node.get_logger().info(f"\n=== RETRY PASS {retry_round}: {len(skipped_indices)} poses remaining ===")

        #     still_failed = []

        #     for i in skipped_indices:
        #         pose = pose_arr[i]
        #         update_marker_color(marker_array, i, r=1.0, g=1.0, b=0.0)
        #         marker_pub.publish(marker_array)

        #         p = pose.pose.position
        #         o = pose.pose.orientation
        #         node.get_logger().info(
        #             f"\n--- [RETRY {retry_round}] Pose {i+1} ---"
        #             f"\n    x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

        #         success = node.plan_to_pose(
        #             p.x, p.y, p.z, o.x, o.y, o.z, o.w,
        #             frame_id=pose.header.frame_id)

        #         if not success:
        #             # Move to random successful waypoint and retry
        #             random_idx = random.choice(completed_indices)
        #             random_pose = pose_arr[random_idx]
        #             rp = random_pose.pose.position
        #             ro = random_pose.pose.orientation
        #             node.get_logger().info(f"Moving to Pose {random_idx+1} before retry...")

        #             if node.plan_to_pose(
        #                 rp.x, rp.y, rp.z, ro.x, ro.y, ro.z, ro.w,
        #                 frame_id=random_pose.header.frame_id):
        #                 node.execute_plan()

        #                 success = node.plan_to_pose(
        #                     p.x, p.y, p.z, o.x, o.y, o.z, o.w,
        #                     frame_id=pose.header.frame_id)

        #         if not success:
        #             still_failed.append(i)
        #             update_marker_color(marker_array, i, r=1.0, g=0.5, b=0.0, a=0.8)
        #             marker_pub.publish(marker_array)
        #             node.get_logger().warn(f"Still failing Pose {i+1}")
        #             continue

        #         if not node.execute_plan():
        #             still_failed.append(i)
        #             update_marker_color(marker_array, i, r=1.0, g=0.0, b=0.0, a=0.8)
        #             marker_pub.publish(marker_array)
        #             node.get_logger().error(f"Execution failed for Pose {i+1}")
        #             continue
        #         completed_indices.append(i)
        #         update_marker_color(marker_array, i, r=0.0, g=1.0, b=0.0)
        #         marker_pub.publish(marker_array)
        #         node.get_logger().info(f"Completed Pose {i+1}")

        #     skipped_indices = still_failed

        node.get_logger().info("Battery scan complete.")

    except KeyboardInterrupt:
        node.get_logger().info("Scan interrupted by user.")
    finally:
        node.remove_collision_plane(_PLANE_ID, FRAME_ID)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
