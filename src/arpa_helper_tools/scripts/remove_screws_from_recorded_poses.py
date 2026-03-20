#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from core_functionality_node import CoreNode
import time
from std_msgs.msg import String
import time
from visualization_msgs.msg import MarkerArray
from custom_ros_messages.srv import RemovePart

def main(args=None):
    rclpy.init(args=args)
    node = CoreNode()
    node.add_collision_plane()

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
        time.sleep(0.5)
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

    # ordered_poses = node.get_tsp_order(pose_stamped_list)
    ordered_poses = pose_stamped_list
    node.get_logger().info(f"{len(ordered_poses)} poses TSP ordered.")
    failures = []
    successes = []
    try:
        start_time = time.time()
        for i, pose in enumerate(ordered_poses):
            req = RemovePart.Request()
            req.visual_servo = False
            req.target_pose.header.frame_id = "floor_link"
            req.target_pose.pose = pose.pose
            future = node.remove_part_client.call_async(req)
            while not future.done():
                time.sleep(0.05)
            result = future.result()
            print(f"RemovePart result: {result.success} — {result.message}")
            result = ""
            print(f"REMOVED {i}")
            # while result not in ["s", "sucess", "f", "failure"]:
            #     result = input(f"results: sucess (s) or failure (f): ")
            # print(f"{result=}")
            # if result in ["s", "sucess"]:
            #     successes.append((i, pose))
            # elif result in ["f", "failure"]:
            #     failures.append((i,pose))
            # else:
            #     print(f"WTF {result=}")
            
            # print("sucesses:")
            # for j, p in successes:
            #     print(f"   {j}:{p}")
            # print("failures:")
            # for j, p in failures:
            #     print(f"   {j}:{p}")
        end_time = time.time()
        total_time = end_time-start_time
        print(f"{total_time=}")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.motor_control(0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
