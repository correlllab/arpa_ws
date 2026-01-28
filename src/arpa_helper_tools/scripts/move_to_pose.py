#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from arpa_control.srv import PlanToPose, ExecutePlan
from custom_ros_messages.srv import EthernetMotor, UR16BehaviorTrigger
from visualization_msgs.msg import MarkerArray


class MoveToPoseNode(Node):
    def __init__(self):
        super().__init__('move_to_pose_node')

        self.plan_client = self.create_client(PlanToPose, 'plan_to_pose')
        self.exec_client = self.create_client(ExecutePlan, 'execute_plan')
        self.motor_client = self.create_client(EthernetMotor, 'motor_control')
        self.behavior_client = self.create_client(UR16BehaviorTrigger, 'ur16e_rest/BehaviorTrigger')

        # Store poses from markers
        self.recorded_poses = []
        self.markers_received = False

        # Subscribe to recorded poses markers
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/recorded_poses_markers',
            self.marker_callback,
            10
        )

        self.get_logger().info("Waiting for plan_to_pose service...")
        self.plan_client.wait_for_service()
        self.get_logger().info("Waiting for execute_plan service...")
        self.exec_client.wait_for_service()
        self.get_logger().info("Waiting for motor_control service...")
        self.motor_client.wait_for_service()
        self.get_logger().info("Waiting for ur16e_rest/BehaviorTrigger service...")
        self.behavior_client.wait_for_service()
        self.get_logger().info("Services ready!")

        self.z_offset = 0.1

    def marker_callback(self, msg):
        if not self.markers_received and len(msg.markers) > 0:
            self.recorded_poses = []
            for marker in msg.markers:
                p = marker.pose.position
                o = marker.pose.orientation
                self.recorded_poses.append((p.x, p.y, p.z + self.z_offset, o.x, o.y, o.z, o.w))
            self.markers_received = True
            self.get_logger().info(f"Received {len(self.recorded_poses)} poses from markers")

    def refresh_markers(self):
        self.markers_received = False
        self.recorded_poses = []
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.markers_received:
                break

    def plan_to_pose(self, x, y, z, qx, qy, qz, qw, frame_id="world"):
        req = PlanToPose.Request()
        req.target_pose.header.frame_id = frame_id
        req.target_pose.pose.position.x = x
        req.target_pose.pose.position.y = y
        req.target_pose.pose.position.z = z
        req.target_pose.pose.orientation.x = qx
        req.target_pose.pose.orientation.y = qy
        req.target_pose.pose.orientation.z = qz
        req.target_pose.pose.orientation.w = qw

        self.get_logger().info(f"Planning to pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                               f"qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f} "
                               f"in frame '{frame_id}'")

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.success:
            self.get_logger().info("Planning successful!")
        else:
            self.get_logger().error(f"Planning failed: {result.message}")
        return result.success

    def execute_plan(self):
        req = ExecutePlan.Request()

        self.get_logger().info("Executing plan...")
        future = self.exec_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.success:
            self.get_logger().info("Execution successful!")
        else:
            self.get_logger().error(f"Execution failed: {result.message}")
        return result.success


    def motor_control(self, speed):
        if not self.motor_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Motor control service not available")
            return False

        req = EthernetMotor.Request()
        req.enable = speed > 0
        req.speed = speed

        action = "ON" if speed > 0 else "OFF"
        self.get_logger().info(f"Motor {action} at speed {speed}")

        future = self.motor_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.success:
            self.get_logger().info(f"Motor: {result.message}")
        else:
            self.get_logger().error(f"Motor failed: {result.message}")
        return result.success

    def trigger_behavior(self, behavior):
        if not self.behavior_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Behavior service not available")
            return False

        req = UR16BehaviorTrigger.Request()
        req.behavior = behavior

        self.get_logger().info(f"Triggering behavior: {behavior}")

        future = self.behavior_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.success:
            self.get_logger().info(f"Behavior: {result.message}")
        else:
            self.get_logger().error(f"Behavior failed: {result.message}")
        return result.success


def main(args=None):
    import sys

    rclpy.init(args=args)
    node = MoveToPoseNode()

    # Wait for markers from record_poses.py
    node.get_logger().info("Waiting for poses from /recorded_poses_markers (run record_poses.py first)...")
    timeout = 10.0
    waited = 0.0
    while not node.markers_received and waited < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
        waited += 0.5

    if not node.markers_received:
        node.get_logger().error("No markers received. Make sure record_poses.py is running with saved poses.")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(f"Connected to record_poses. {len(node.recorded_poses)} poses available.")

    node.motor_control(0)
    try:
        for pose in node.recorded_poses:
            x, y, z, qx, qy, qz, qw = pose
            plan_approved = False
            while not plan_approved:
                node.plan_to_pose(x, y, z, qx, qy, qz, qw)
                user_input = input(f"press e to approve plan: ")
                if user_input.lower() == 'e':
                    plan_approved = True
            node.execute_plan()    
            node.get_logger().info("Triggering zforce behavior...")
            node.trigger_behavior("zforce")
            node.get_logger().info("Zforce behavior completed.")
            node.trigger_behavior("play")
            time.sleep(1)
            node.get_logger().info("Triggering retract behavior...")
            node.trigger_behavior("retract")
            node.get_logger().info("Retract behavior completed.")
            node.trigger_behavior("play")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.motor_control(0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
