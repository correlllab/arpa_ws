#!/usr/bin/env python3

import json
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from core_functionality_node import CoreNode
import time


def main(args=None):
    #create everything we need
    rclpy.init(args=args)
    node = CoreNode()
    node.visualize_detections()
    node.add_collision_plane("battery_do_not_cross", "floor_link", 0.118, -0.056, 0.9, 2.182, 1.574)
    behavior_publisher = node.create_publisher(String, '/triggered_behavior', 10)
    capture_client = node.create_client(Trigger, 'record_images/capture')

    REMOVAL_BEHAVIOR = "ZForce"#SpiralForce" #"SpiralForce" or ZForce 
    SECOND_LOOK = True
    SAVE_IMAGES = False
    
    
    
    
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
    for label, centroid in detections.items():
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
            plan_successful = False
            tries = 0
            behavior_publisher.publish(String(data=f"planning_to_{label}"))
            while not plan_successful and tries < 3:
                tries += 1
                success = node.plan_toolhead_to_pose(x, y, 0.91, qx, qy, qz, qw)
                if success:
                    plan_successful = True
                    node.get_logger().info("Planning succeeded!")
                else:
                    node.get_logger().warn(f"Planning failed, retrying... (attempt {tries}/3)")
            if not plan_successful:
                node.get_logger().error("Failed to plan after 3 attempts, skipping this target.")
                continue       
            
            behavior_publisher.publish(String(data="executing_plan"))
            node.execute_plan()
            time.sleep(1)

            if SECOND_LOOK:
                behavior_publisher.publish(String(data="second sight"))
                screw_pose = PoseStamped()
                screw_pose.header = pose.header
                screw_pose.pose.position.x = pose.pose.position.x
                screw_pose.pose.position.y = pose.pose.position.y
                screw_pose.pose.position.z = pose.pose.position.z
                screw_pose.pose.orientation = pose.pose.orientation
                alignment_result = node.align_to_screw_img(initial_screw_pose=screw_pose)
                node.get_logger().info(f"Second sight done final result={alignment_result}. Proceeding with removal (screw may be occluded by EE when aligned).")

            if capture_client.service_is_ready() and SAVE_IMAGES:
                future = capture_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                if future.done():
                    node.get_logger().info(f"  Captured img: {future.result().message}")
                else:
                    node.get_logger().warn(f"  Capture timed out at orientation")
            else:
                node.get_logger().warn(f"  Capture service not available at orientation or SAVE_IMAGES is false {SAVE_IMAGES=}, skipping")


            behavior_publisher.publish(String(data="removal"))
            node.motor_control(100)
            trigger_with_retry(node, REMOVAL_BEHAVIOR)
            trigger_with_retry(node, "play")
            time.sleep(2)

            behavior_publisher.publish(String(data="retracting")) 
            trigger_with_retry(node, "retract")
            trigger_with_retry(node, "play")
            time.sleep(2)
            behavior_publisher.publish(String(data="retracted finishes"))


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
