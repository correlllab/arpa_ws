#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arpa_control.srv import PlanToPose, ExecutePlan
from custom_ros_messages.srv import EthernetMotor, UR16BehaviorTrigger
from std_srvs.srv import Trigger
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class CoreNode(Node):
    def __init__(self):
        super().__init__('move_to_pose_node')

        self.plan_client = self.create_client(PlanToPose, 'plan_to_pose')
        self.exec_client = self.create_client(ExecutePlan, 'execute_plan')
        self.motor_client = self.create_client(EthernetMotor, 'motor_control')
        self.behavior_client = self.create_client(UR16BehaviorTrigger, 'ur16e_rest/BehaviorTrigger')
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory'
        )
        self.update_depth_client = self.create_client(Trigger, 'update_depth')
        self.planning_scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

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

    def update_depth(self):
        if not self.update_depth_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Update depth service not available")
            return False

        req = Trigger.Request()
        self.get_logger().info("Updating depth map...")

        future = self.update_depth_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.success:
            self.get_logger().info(f"Depth update: {result.message}")
        else:
            self.get_logger().error(f"Depth update failed: {result.message}")
        return result.success

    def add_collision_plane(self, plane_id, frame_id, x, y, z, size_x, size_y, thickness=0.02):
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = plane_id
        collision_object.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [size_x, size_y, thickness]

        box_pose = Pose()
        box_pose.position.x = x
        box_pose.position.y = y
        box_pose.position.z = z
        box_pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)

        self.planning_scene_pub.publish(planning_scene)
        self.get_logger().info(f"Added collision plane '{plane_id}' at z={z}")

    def remove_collision_plane(self, plane_id, frame_id):
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = plane_id
        collision_object.operation = CollisionObject.REMOVE

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)

        self.planning_scene_pub.publish(planning_scene)
        self.get_logger().info(f"Removed collision plane '{plane_id}'")

    def go_home(self, frame_id="floor_link"):
        self.get_logger().info("Going home...")

        if self.plan_to_pose(1.112, -0.573, 1.253, 0.7071068, 0.7071068, 0.0, 0.0, frame_id=frame_id):
            return self.execute_plan()
        self.get_logger().error("Failed to plan home position")
        return False

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


def print_menu():
    print("\n=== ARPA Core Control ===")
    print("1. Go home")
    print("2. Remove collision plane")
    print("3. Trigger behavior")
    print("4. Update depth")
    print("5. Motor control")
    print("0. Quit")
    print("========================")


def main(args=None):
    rclpy.init(args=args)
    node = CoreNode()

    node.add_collision_plane("battery_do_not_cross", "floor_link", 0.118, -0.056, 0.9, 2.182, 1.574)

    try:
        while True:
            print_menu()
            choice = input("Select: ").strip()

            if choice == "1":
                node.go_home()

            elif choice == "2":
                plane_id = input("Plane ID [battery_do_not_cross]: ").strip() or "battery_do_not_cross"
                frame_id = input("Frame ID [floor_link]: ").strip() or "floor_link"
                node.remove_collision_plane(plane_id, frame_id)
                return
            elif choice == "3":
                behavior = input("Behavior name: ").strip()
                if behavior:
                    node.trigger_behavior(behavior)

            elif choice == "4":
                node.update_depth()

            elif choice == "5":
                speed = int(input("Speed (0=off): ").strip() or "0")
                node.motor_control(speed)

            elif choice == "0":
                break

            else:
                print("Invalid choice")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
