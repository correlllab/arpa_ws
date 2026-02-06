#!/usr/bin/env python3
"""
Battery Scanning Helper Script

Iterates through predefined scanning poses in a serpentine pattern.
For each pose: plans the motion, waits for user approval, then executes.
Uses MoveToPoseNode from move_to_pose.py for planning and execution.
"""

import rclpy
from move_to_pose import MoveToPoseNode
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import (
    CollisionObject, PlanningScene, RobotState,
    Constraints, PositionConstraint, OrientationConstraint,
    BoundingVolume, MotionPlanRequest
)
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
import time
import random


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
SCAN_POSES = []
for row_idx, x_pos in enumerate(_X_POSITIONS):
    y_range = _Y_POSITIONS if row_idx % 2 == 0 else list(reversed(_Y_POSITIONS))

    for col_idx, y_pos in enumerate(y_range):
        pose_num = row_idx * N_Y_STEPS + col_idx + 1

        SCAN_POSES.append({
            "name": f"Pose {pose_num}",
            "x": x_pos,
            "y": y_pos,
            "z": _Z_HEIGHT,
            "qx": _QX,
            "qy": _QY,
            "qz": _QZ,
            "qw": _QW
        })

# Reorder poses: first half in order, second half in reverse
mid = len(SCAN_POSES) // 2
SCAN_POSES = SCAN_POSES[:mid] + list(reversed(SCAN_POSES[mid:]))


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


def add_collision_plane(node, planning_scene_pub):
    """Add a collision plane below the battery scan area."""
    collision_object = CollisionObject()
    collision_object.header.frame_id = FRAME_ID
    collision_object.header.stamp = node.get_clock().now().to_msg()
    collision_object.id = _PLANE_ID
    collision_object.operation = CollisionObject.ADD

    # Define a thin box as the plane
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [_PLANE_SIZE_X, _PLANE_SIZE_Y, _PLANE_THICKNESS]

    box_pose = Pose()
    box_pose.position.x = _PLANE_CENTER_X
    box_pose.position.y = _PLANE_CENTER_Y
    box_pose.position.z = _PLANE_Z
    box_pose.orientation.w = 1.0

    collision_object.primitives.append(box)
    collision_object.primitive_poses.append(box_pose)

    # Publish via PlanningScene
    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    planning_scene.world.collision_objects.append(collision_object)

    planning_scene_pub.publish(planning_scene)
    node.get_logger().info(f"Added collision plane '{_PLANE_ID}' at z={_PLANE_Z}")


def remove_collision_plane(node, planning_scene_pub):
    """Remove the collision plane from the planning scene."""
    collision_object = CollisionObject()
    collision_object.header.frame_id = FRAME_ID
    collision_object.header.stamp = node.get_clock().now().to_msg()
    collision_object.id = _PLANE_ID
    collision_object.operation = CollisionObject.REMOVE

    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    planning_scene.world.collision_objects.append(collision_object)

    planning_scene_pub.publish(planning_scene)
    node.get_logger().info(f"Removed collision plane '{_PLANE_ID}'")


def get_current_joint_state(node, timeout_sec=2.0):
    """Get the current joint state from /joint_states topic."""
    joint_state_msg = None

    def callback(msg):
        nonlocal joint_state_msg
        joint_state_msg = msg

    sub = node.create_subscription(JointState, '/joint_states', callback, 10)

    # Wait for a message
    start_time = node.get_clock().now()
    while joint_state_msg is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed > timeout_sec:
            node.get_logger().error('Timeout waiting for joint state')
            break

    node.destroy_subscription(sub)
    return joint_state_msg


def compute_cartesian_path(node, waypoints, max_step=0.01, jump_threshold=2.0):
    """
    Compute a Cartesian path through the given waypoints.

    Args:
        node: ROS2 node with the service client
        waypoints: List of PoseStamped waypoints
        max_step: Maximum step size between interpolated points (meters)
        jump_threshold: Maximum allowed jump in joint space (0.0 disables check)

    Returns:
        (trajectory, fraction) tuple where:
            - trajectory: RobotTrajectory message (or None on failure)
            - fraction: Path completion ratio 0.0-1.0
                1.0 = full path computed successfully
                <1.0 = path partially computed (hit obstacle or joint limit)
                0.0 = no valid path found
    """
    client = node.create_client(GetCartesianPath, '/compute_cartesian_path')

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Cartesian path service not available')
        return None, 0.0

    # Get current robot state
    joint_state = get_current_joint_state(node)
    if joint_state is None:
        node.get_logger().error('Failed to get current joint state')
        return None, 0.0

    request = GetCartesianPath.Request()
    request.header.frame_id = FRAME_ID
    request.header.stamp = node.get_clock().now().to_msg()
    request.group_name = 'ur16e_on_gantry'
    request.link_name = 'tool0'

    # Set the start state to current robot position
    request.start_state = RobotState()
    request.start_state.joint_state = joint_state

    request.waypoints = [wp.pose for wp in waypoints]
    request.max_step = max_step
    request.jump_threshold = jump_threshold
    request.avoid_collisions = True

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        node.get_logger().error('Cartesian path service call failed')
        return None, 0.0

    response = future.result()
    return response.solution, response.fraction


def build_waypoints_from_poses(node, pose_list):
    """Convert scan pose dicts to PoseStamped waypoints."""
    waypoints = []
    for pose in pose_list:
        ps = PoseStamped()
        ps.header.frame_id = FRAME_ID
        ps.header.stamp = node.get_clock().now().to_msg()
        ps.pose.position.x = pose['x']
        ps.pose.position.y = pose['y']
        ps.pose.position.z = pose['z']
        ps.pose.orientation.x = pose['qx']
        ps.pose.orientation.y = pose['qy']
        ps.pose.orientation.z = pose['qz']
        ps.pose.orientation.w = pose['qw']
        waypoints.append(ps)
    return waypoints


def build_scan_marker_array(node):
    """Build a MarkerArray with an arrow marker for each scan pose."""
    marker_array = MarkerArray()
    for i, pose in enumerate(SCAN_POSES):
        m = Marker()
        m.header.frame_id = FRAME_ID
        m.header.stamp = node.get_clock().now().to_msg()
        m.ns = "scan_poses"
        m.id = i
        m.type = Marker.ARROW
        m.action = Marker.ADD

        m.pose.position.x = pose['x']
        m.pose.position.y = pose['y']
        m.pose.position.z = pose['z']
        m.pose.orientation.x = pose['qx']
        m.pose.orientation.y = pose['qy']
        m.pose.orientation.z = pose['qz']
        m.pose.orientation.w = pose['qw']

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
    node = MoveToPoseNode()

    marker_pub = node.create_publisher(MarkerArray, '/scan_poses_markers', 10)
    planning_scene_pub = node.create_publisher(PlanningScene, '/planning_scene', 10)
    marker_array = build_scan_marker_array(node)

    # Give publishers time to connect
    time.sleep(1.0)

    # Publish markers so they appear in rviz
    marker_pub.publish(marker_array)

    # Add collision plane to prevent planner from going below battery
    add_collision_plane(node, planning_scene_pub)

    # Publish markers again to ensure visibility
    marker_pub.publish(marker_array)

    node.get_logger().info(f"Battery scan: {len(SCAN_POSES)} poses to visit")
    CARTESIAN_MOVE = False

    try:
        if CARTESIAN_MOVE:
            # === CARTESIAN PATH MODE ===
            waypoints = build_waypoints_from_poses(node, SCAN_POSES)

            # Highlight all poses in yellow (planning)
            for i in range(len(SCAN_POSES)):
                update_marker_color(marker_array, i, r=1.0, g=1.0, b=0.0)
            marker_pub.publish(marker_array)

            # Plan/replan loop with user confirmation
            while True:
                trajectory, fraction = compute_cartesian_path(node, waypoints)

                if trajectory is None:
                    user_input = input("Path computation failed. [r]etry / [q]uit: ").strip().lower()
                    if user_input == 'q':
                        node.get_logger().info("Cancelled by user.")
                        return
                    continue

                pct = fraction * 100
                n_points = len(trajectory.joint_trajectory.points)

                if fraction < 1.0:
                    node.get_logger().warn(f'Partial path: {pct:.1f}% achieved ({n_points} points)')
                else:
                    node.get_logger().info(f'Full path computed: {pct:.1f}% ({n_points} points)')

                user_input = input(f"Path ready ({pct:.0f}%). [e]xecute / [r]eplan / [q]uit: ").strip().lower()

                if user_input == 'e':
                    break
                elif user_input == 'q':
                    node.get_logger().info("Cancelled by user.")
                    return
                # else replan

            # Execute the trajectory
            node.get_logger().info("Executing cartesian path...")
            node.execute_trajectory(trajectory)

            # Mark poses based on fraction achieved
            completed_count = int(fraction * len(SCAN_POSES))
            for i in range(len(SCAN_POSES)):
                if i < completed_count:
                    update_marker_color(marker_array, i, r=0.0, g=1.0, b=0.0)
                else:
                    update_marker_color(marker_array, i, r=0.5, g=0.5, b=0.5, a=0.4)
            marker_pub.publish(marker_array)

        else:
            # === POSE-BY-POSE MODE ===
            skipped_indices = []  # Track skipped poses for retry
            completed_indices = []  # Track successful poses

            # First pass: visit all poses, skip failures
            for i, pose in enumerate(SCAN_POSES):
                # Highlight current pose in yellow
                update_marker_color(marker_array, i, r=1.0, g=1.0, b=0.0)
                marker_pub.publish(marker_array)

                node.get_logger().info(
                    f"\n--- [{i+1}/{len(SCAN_POSES)}] {pose['name']} ---"
                    f"\n    x={pose['x']:.3f}, y={pose['y']:.3f}, z={pose['z']:.3f}")

                success = node.plan_to_pose(
                    pose['x'], pose['y'], pose['z'],
                    pose['qx'], pose['qy'], pose['qz'], pose['qw'],
                    frame_id=FRAME_ID)

                if not success:
                    # Auto-skip on failure, mark for retry
                    skipped_indices.append(i)
                    update_marker_color(marker_array, i, r=1.0, g=0.5, b=0.0, a=0.8)  # Orange = deferred
                    marker_pub.publish(marker_array)
                    node.get_logger().warn(f"Skipping {pose['name']} (will retry later)")
                    continue

                # Auto-execute
                node.execute_plan()
                completed_indices.append(i)
                update_marker_color(marker_array, i, r=0.0, g=1.0, b=0.0)
                marker_pub.publish(marker_array)
                node.get_logger().info(f"Completed {pose['name']}")

            # Retry passes: keep retrying from random successful waypoints
            retry_round = 0
            while skipped_indices and completed_indices:
                retry_round += 1
                node.get_logger().info(f"\n=== RETRY PASS {retry_round}: {len(skipped_indices)} poses remaining ===")

                still_failed = []

                for i in skipped_indices:
                    pose = SCAN_POSES[i]
                    update_marker_color(marker_array, i, r=1.0, g=1.0, b=0.0)
                    marker_pub.publish(marker_array)

                    node.get_logger().info(
                        f"\n--- [RETRY {retry_round}] {pose['name']} ---"
                        f"\n    x={pose['x']:.3f}, y={pose['y']:.3f}, z={pose['z']:.3f}")

                    # Try planning from current position
                    success = node.plan_to_pose(
                        pose['x'], pose['y'], pose['z'],
                        pose['qx'], pose['qy'], pose['qz'], pose['qw'],
                        frame_id=FRAME_ID)

                    if not success:
                        # Move to random successful waypoint and retry
                        random_idx = random.choice(completed_indices)
                        random_pose = SCAN_POSES[random_idx]
                        node.get_logger().info(f"Moving to {random_pose['name']} before retry...")

                        if node.plan_to_pose(
                            random_pose['x'], random_pose['y'], random_pose['z'],
                            random_pose['qx'], random_pose['qy'], random_pose['qz'], random_pose['qw'],
                            frame_id=FRAME_ID):
                            node.execute_plan()

                            # Try failed pose again
                            success = node.plan_to_pose(
                                pose['x'], pose['y'], pose['z'],
                                pose['qx'], pose['qy'], pose['qz'], pose['qw'],
                                frame_id=FRAME_ID)

                    if not success:
                        still_failed.append(i)
                        update_marker_color(marker_array, i, r=1.0, g=0.5, b=0.0, a=0.8)
                        marker_pub.publish(marker_array)
                        node.get_logger().warn(f"Still failing {pose['name']}")
                        continue

                    # Auto-execute
                    node.execute_plan()
                    completed_indices.append(i)
                    update_marker_color(marker_array, i, r=0.0, g=1.0, b=0.0)
                    marker_pub.publish(marker_array)
                    node.get_logger().info(f"Completed {pose['name']}")

                skipped_indices = still_failed

        node.get_logger().info("Battery scan complete.")

    except KeyboardInterrupt:
        node.get_logger().info("Scan interrupted by user.")
    finally:
        remove_collision_plane(node, planning_scene_pub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
