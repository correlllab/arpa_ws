#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arpa_control.srv import PlanToPose, ExecutePlan
from custom_ros_messages.srv import EthernetMotor, UR16BehaviorTrigger
from moveit_msgs.action import ExecuteTrajectory


class MoveToPoseNode(Node):
    def __init__(self):
        super().__init__('move_to_pose_node')

        self.plan_client = self.create_client(PlanToPose, 'plan_to_pose')
        self.exec_client = self.create_client(ExecutePlan, 'execute_plan')
        self.motor_client = self.create_client(EthernetMotor, 'motor_control')
        self.behavior_client = self.create_client(UR16BehaviorTrigger, 'ur16e_rest/BehaviorTrigger')
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory'
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

    def execute_trajectory(self, trajectory):
        """Execute a RobotTrajectory directly via MoveIt's ExecuteTrajectory action."""
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ExecuteTrajectory action server not available")
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info("Sending trajectory for execution...")
        future = self.execute_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory execution goal rejected")
            return False

        self.get_logger().info("Trajectory accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Trajectory execution successful!")
            return True
        else:
            self.get_logger().error(f"Trajectory execution failed: error_code={result.error_code.val}")
            return False

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
    import time
    from visualization_msgs.msg import MarkerArray

    rclpy.init(args=args)
    node = MoveToPoseNode()

    # Marker subscription state (local to main)
    recorded_poses = []
    markers_received = False
    z_offset = 0.1

    def marker_callback(msg):
        nonlocal recorded_poses, markers_received
        if not markers_received and len(msg.markers) > 0:
            recorded_poses = []
            for marker in msg.markers:
                p = marker.pose.position
                o = marker.pose.orientation
                recorded_poses.append((p.x, p.y, p.z + z_offset, o.x, o.y, o.z, o.w))
            markers_received = True
            node.get_logger().info(f"Received {len(recorded_poses)} poses from markers")

    marker_sub = node.create_subscription(
        MarkerArray, '/recorded_poses_markers', marker_callback, 10)

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

    node.motor_control(0)
    try:
        for pose in recorded_poses:
            x, y, z, qx, qy, qz, qw = pose
            plan_approved = False
            while not plan_approved:
                node.plan_to_pose(x, y, z, qx, qy, qz, qw)
                user_input = input("press e to approve plan: ")
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
