#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from core_functionality_node import CoreNode
import time

def trigger_with_retry(node, behavior, retries=3):
    for attempt in range(1, retries + 1):
        if node.trigger_behavior(behavior):
            return True
        node.get_logger().warn(f"Behavior '{behavior}' failed (attempt {attempt}/{retries}), retrying...")
        time.sleep(1)
    node.get_logger().error(f"Behavior '{behavior}' failed after {retries} attempts.")
    return False


def main(args=None):
    import time
    from visualization_msgs.msg import MarkerArray

    rclpy.init(args=args)
    node = CoreNode()
    node.add_collision_plane("battery_do_not_cross", "floor_link", 0.118, -0.056, 0.9, 2.182, 1.574)


    # Marker subscription state (local to main)
    recorded_poses = []
    markers_received = False

    def marker_callback(msg):
        nonlocal recorded_poses, markers_received
        if not markers_received and len(msg.markers) > 0:
            recorded_poses = []
            for marker in msg.markers:
                p = marker.pose.position
                o = marker.pose.orientation
                recorded_poses.append((p.x, p.y, p.z, o.x, o.y, o.z, o.w))
            markers_received = True
            node.get_logger().info(f"Received {len(recorded_poses)} poses from markers")

    marker_sub = node.create_subscription(MarkerArray, '/recorded_poses_markers', marker_callback, 10)

    # Wait for markers from record_poses.py
    node.get_logger().info("Waiting for poses from /recorded_poses_markers (run record_poses.py first)...")
    timeout = 10.0
    waited = 0.0
    while not markers_received and waited < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
        waited += 0.5

    if not markers_received:
        node.get_logger().error("No markers received. Make sure record_poses.py is running with saved poses.")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(f"Connected to record_poses. {len(recorded_poses)} poses available.")

    # Convert to PoseStamped and get TSP order
    pose_stamped_list = []
    for x, y, z, qx, qy, qz, qw in recorded_poses:
        ps = PoseStamped()
        ps.header.frame_id = 'floor_link'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        pose_stamped_list.append(ps)

    ordered_poses = node.get_tsp_order(pose_stamped_list)
    node.get_logger().info(f"{len(ordered_poses)} poses TSP ordered.")

    try:
        for pose in ordered_poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            qx = pose.pose.orientation.x
            qy = pose.pose.orientation.y
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w
            plan_successful = False
            while not plan_successful:
                success, _ = node.plan_to_pose(x, y, z, qx, qy, qz, qw)
                if success:
                    plan_successful = True
                    node.get_logger().info("Planning succeeded!")
                else:
                    node.get_logger().warn("Planning failed, retrying...")
            node.execute_plan()

            trigger_with_retry(node, "zforce")
            node.motor_control(100)
            trigger_with_retry(node, "play")
            time.sleep(2)

            trigger_with_retry(node, "retract")
            trigger_with_retry(node, "play")
            time.sleep(1)

            node.motor_control(0)
            trigger_with_retry(node, "ros2control")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.motor_control(0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
