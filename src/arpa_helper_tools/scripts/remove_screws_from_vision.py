#!/usr/bin/env python3

import json
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from core_functionality_node import CoreNode
import time
from custom_ros_messages.srv import RemovePart

def main(args=None):
    #create everything we need
    rclpy.init(args=args)
    node = CoreNode()
    node.visualize_detections()
    node.add_collision_plane()

    SECOND_LOOK = False
    
    # Get detections from vision node
    client = node.create_client(Trigger, '/arpa_vision_node/get_detections_json')
    node.get_logger().info("Waiting for /arpa_vision_node/get_detections_json service...")
    client.wait_for_service(timeout_sec=10.0)
    future = client.call_async(Trigger.Request())
    while not future.done():
        time.sleep(0.05)
    response = future.result()
    if not response.success:
        node.get_logger().error(f"Vision service failed: {response.message}")
        node.destroy_node()
        rclpy.shutdown()
        return
    detections = json.loads(response.message)
    # node.get_logger().info(f"Got detections: {detections}")
    
    
    # Convert centroids to PoseStamped for TSP ordering
    pose_stamped_list = []
    label_list = []
    for label, (centroid, confidence) in detections.items():
        if not any(label.lower().split('_')[0] == k for k in ('nut', 'screw')):
            continue
        cx, cy, cz = centroid

        # Build orientation: tool Z down, tool Y toward origin in XY plane
        z_hat = np.array([0.0, 0.0, -1.0])
        toward_origin = np.array([-cx, -cy, 0.0])
        norm = np.linalg.norm(toward_origin)
        y_hat = toward_origin / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0])
        x_hat = np.cross(y_hat, z_hat)
        R = np.column_stack([x_hat, y_hat, z_hat])
        qx, qy, qz, qw = Rotation.from_matrix(R).as_quat()

        ps = PoseStamped()
        ps.header.frame_id = 'floor_link'
        ps.pose.position.x = cx
        ps.pose.position.y = cy
        ps.pose.position.z = cz
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        pose_stamped_list.append(ps)
        label_list.append(label)
        node.get_logger().info(f"Added pose for {label}: position=({cx:.3f}, {cy:.3f}, {cz:.3f}), orientation=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})\n\n")

    ordered_poses = node.get_tsp_order(pose_stamped_list)
    _pose_id_to_label = {id(p): label_list[i] for i, p in enumerate(pose_stamped_list)}
    ordered_labels = [_pose_id_to_label[id(p)] for p in ordered_poses]
    node.get_logger().info(f"Got {len(ordered_poses)} poses from vision (TSP ordered).")

    

    try:
        for label, pose in zip(ordered_labels, ordered_poses):
            node.get_logger().info(f"\n\nProcessing {label} at position=({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})")

            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            qx = pose.pose.orientation.x
            qy = pose.pose.orientation.y
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w


            req = RemovePart.Request()
            req.part_name = label
            req.detection_confidence = confidence
            req.visual_servo = SECOND_LOOK
            req.target_pose.header.frame_id = "floor_link"
            req.target_pose.pose.position.x = x
            req.target_pose.pose.position.y = y
            req.target_pose.pose.position.z = z
            req.target_pose.pose.orientation.x = qx
            req.target_pose.pose.orientation.y = qy
            req.target_pose.pose.orientation.z = qz
            req.target_pose.pose.orientation.w = qw
            future = node.remove_part_client.call_async(req)
            while not future.done():
                time.sleep(0.05)
            result = future.result()
            print(f"RemovePart result: {result.success} — {result.message}")

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.motor_control(0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
