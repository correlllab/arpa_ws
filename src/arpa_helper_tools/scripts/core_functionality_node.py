#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from arpa_control.srv import PlanToPose, ExecutePlan, GetPoseCostMatrix
from custom_ros_messages.srv import EthernetMotor, UR16BehaviorTrigger
from std_srvs.srv import Trigger
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import threading
import time
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

BASE_FRAME = "floor_link"
EE_FRAME = "wrist_3_link"




class CoreNode(Node):
    def __init__(self):
        super().__init__('core_functionality_node')

        self.plan_client = self.create_client(PlanToPose, 'plan_to_pose')
        self.exec_client = self.create_client(ExecutePlan, 'execute_plan')
        self.motor_client = self.create_client(EthernetMotor, 'motor_control')
        self.behavior_client = self.create_client(UR16BehaviorTrigger, 'ur16e_rest/BehaviorTrigger')
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory'
        )
        self.update_depth_client = self.create_client(Trigger, 'update_depth')
        self.pose_cost_matrix_client = self.create_client(GetPoseCostMatrix, 'get_pose_cost_matrix')
        self.planning_scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # Required services (benchmark should fail fast if these aren't up)
        self.get_logger().info("Waiting for plan_to_pose service...")
        if not self.plan_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("Timed out waiting for plan_to_pose service")
        self.get_logger().info("Waiting for execute_plan service...")
        if not self.exec_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("Timed out waiting for execute_plan service")

        # Optional services (available on real robot / full stack; skip in sim if missing)
        self.get_logger().info("Checking optional services...")
        self._motor_available = self.motor_client.wait_for_service(timeout_sec=2.0)
        if not self._motor_available:
            self.get_logger().warn("motor_control service not available (continuing without motor control)")

        self._behavior_available = self.behavior_client.wait_for_service(timeout_sec=2.0)
        if not self._behavior_available:
            self.get_logger().warn("ur16e_rest/BehaviorTrigger service not available (continuing without behavior triggers)")

        self._update_depth_available = self.update_depth_client.wait_for_service(timeout_sec=2.0)
        if not self._update_depth_available:
            self.get_logger().warn("update_depth service not available (continuing without depth updates)")

        self._pose_cost_matrix_available = self.pose_cost_matrix_client.wait_for_service(timeout_sec=5.0)
        if not self._pose_cost_matrix_available:
            self.get_logger().warn("get_pose_cost_matrix service not available (will use pose list order without TSP optimization)")

        self.get_logger().info("Core services ready!")
        self.last_plan_num_ik_solutions = 0
        self.last_plan_selected_ik_solution_index = -1


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Spin in a background thread to keep TF buffer up to date
        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self._spin_thread.start()

        self.get_logger().info("Looking up wrist_3_link -> tool_head_link transform...")
        self.T_wrist3_to_toolhead = None
        while self.T_wrist3_to_toolhead is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'tool_head_link', 'wrist_3_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                t = tf.transform.translation
                r = tf.transform.rotation
                from scipy.spatial.transform import Rotation
                rot = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
                mat = np.eye(4)
                mat[:3, :3] = rot
                mat[:3,  3] = [t.x, t.y, t.z]
                self.T_wrist3_to_toolhead = mat
                self.get_logger().info(f"wrist_3_link -> tool_head_link:\n{mat}")
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"TF not ready yet: {e}. Retrying...")
                time.sleep(0.5)
        self.T_toolhead_to_wrist3 = np.linalg.inv(self.T_wrist3_to_toolhead)

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
        while not future.done():
            time.sleep(0.05)

        result = future.result()
        self.last_plan_num_ik_solutions = result.num_ik_solutions
        self.last_plan_selected_ik_solution_index = result.selected_ik_solution_index
        if result.success:
            self.get_logger().info(
                f"Planning successful! (manipulability={result.manipulability_score:.6f}, "
                f"ik_solutions={result.num_ik_solutions}, "
                f"selected={result.selected_ik_solution_index})"
            )
        else:
            self.get_logger().error(f"Planning failed: {result.message}")
        return result.success, result.manipulability_score

    def plan_toolhead_to_pose(self, x, y, z, qx, qy, qz, qw, frame_id="world"):
        # Convert toolhead pose to wrist_3_link pose using the known transform
        target_toolhead = np.eye(4)
        target_toolhead[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        target_toolhead[:3, 3] = [x, y, z]
        # target_wrist3 = target_toolhead @ self.T_toolhead_to_wrist3
        target_wrist3 = target_toolhead @ self.T_wrist3_to_toolhead 


        wx, wy, wz = target_wrist3[:3, 3]
        rot = target_wrist3[:3, :3]
        qx, qy, qz, qw = Rotation.from_matrix(rot).as_quat()

        return self.plan_to_pose(wx, wy, wz, qx, qy, qz, qw, frame_id)

    def execute_plan(self):
        req = ExecutePlan.Request()

        self.get_logger().info("Executing plan...")
        future = self.exec_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

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
        while not future.done():
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory execution goal rejected")
            return False

        self.get_logger().info("Trajectory accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        result = result_future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Trajectory execution successful!")
            return True
        else:
            self.get_logger().error(f"Trajectory execution failed: error_code={result.error_code.val}")
            return False

    def motor_control(self, speed):
        if not getattr(self, "_motor_available", False):
            self.get_logger().warn("Motor control requested but motor_control service is unavailable (skipping)")
            return False
        if not self.motor_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Motor control service not available")
            return False

        req = EthernetMotor.Request()
        req.enable = speed > 0
        req.speed = speed

        action = "ON" if speed > 0 else "OFF"
        self.get_logger().info(f"Motor {action} at speed {speed}")

        future = self.motor_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

        result = future.result()
        if result.success:
            self.get_logger().info(f"Motor: {result.message}")
        else:
            self.get_logger().error(f"Motor failed: {result.message}")
        return result.success

    def update_depth(self):
        if not getattr(self, "_update_depth_available", False):
            self.get_logger().warn("Depth update requested but update_depth service is unavailable (skipping)")
            return False
        if not self.update_depth_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Update depth service not available")
            return False

        req = Trigger.Request()
        self.get_logger().info("Updating depth map...")

        future = self.update_depth_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

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

    def go_home(self, frame_id="floor_link", toolhead=False):
        self.get_logger().info("Going home...")
        plan_success = False
        exec_success = False
        if toolhead:
            plan_success, _ = self.plan_toolhead_to_pose(1.112, -0.573, 1.253, 0.7071068, 0.7071068, 0.0, 0.0, frame_id=frame_id)
            exec_success = self.execute_plan()
        else:
            plan_success, _ = self.plan_to_pose(1.112, -0.573, 1.253, 0.7071068, 0.7071068, 0.0, 0.0, frame_id=frame_id)
            exec_success = self.execute_plan()
        self.get_logger().error(f"go home {plan_success=}, {exec_success=}")
        return plan_success and exec_success

    def trigger_behavior(self, behavior):
        if not getattr(self, "_behavior_available", False):
            self.get_logger().warn(f"Behavior '{behavior}' requested but BehaviorTrigger service is unavailable (skipping)")
            return False
        if not self.behavior_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Behavior service not available")
            return False

        req = UR16BehaviorTrigger.Request()
        req.behavior = behavior

        self.get_logger().info(f"Triggering behavior: {behavior}")

        future = self.behavior_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

        result = future.result()
        if result.success:
            self.get_logger().info(f"Behavior: {result.message}")
        else:
            self.get_logger().error(f"Behavior failed: {result.message}")
        return result.success


    def get_tsp_order(self, poses, euclidean=True):
        """
        poses: list of PoseStamped

        returns: list of PoseStamped in optimal visit order from current robot position
        """
        if not getattr(self, "_pose_cost_matrix_available", False):
            return poses

        # Get current EE pose as start node (required for TSP). If TF not ready, use pose order.
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                EE_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF not available for TSP ({e}). Using pose list order."
            )
            return poses

        start_pose = PoseStamped()
        start_pose.header.frame_id = BASE_FRAME
        start_pose.pose.position.x = transform.transform.translation.x
        start_pose.pose.position.y = transform.transform.translation.y
        start_pose.pose.position.z = transform.transform.translation.z
        start_pose.pose.orientation = transform.transform.rotation
        self.get_logger().info(
            f"Current EE position: ({start_pose.pose.position.x:.3f}, "
            f"{start_pose.pose.position.y:.3f}, {start_pose.pose.position.z:.3f})")

        all_poses = [start_pose] + list(poses)
        n = len(all_poses)
        dummy_end_idx = n  # virtual node — no physical location

        # Get full pairwise cost matrix in a single service call
        self.get_logger().info(f"Requesting {n}x{n} cost matrix from service...")
        req = GetPoseCostMatrix.Request()
        req.poses = all_poses
        req.euclidean = euclidean
        start_time = time.time()
        future = self.pose_cost_matrix_client.call_async(req)
        while not future.done():
            time.sleep(0.05)
        end_time = time.time()

        result = future.result()
        self.get_logger().info(f"Cost matrix computed in {end_time - start_time:.2f} seconds")
        if not result.success:
            self.get_logger().error(f"GetPoseCostMatrix failed: {result.message}")
            raise ValueError(f"GetPoseCostMatrix failed: {result.message}")

        self.get_logger().info(f"Cost matrix received: {result.message}")

        # Reshape flat row-major array into 2D cost matrix, add dummy end column/row
        flat = result.cost_matrix
        cost_matrix = [[0.0] * (n + 1) for _ in range(n + 1)]
        for i in range(n):
            for j in range(n):
                cost_matrix[i][j] = flat[i * n + j]

        manager = pywrapcp.RoutingIndexManager(
            n + 1,          # nodes: start + targets + dummy end
            1,              # one vehicle
            [0],            # start depot
            [dummy_end_idx] # end depot (dummy)
        )
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            i = manager.IndexToNode(from_index)
            j = manager.IndexToNode(to_index)
            if i == dummy_end_idx or j == dummy_end_idx:
                return 0
            return int(cost_matrix[i][j] * 1000)

        cb_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(cb_index)

        params = pywrapcp.DefaultRoutingSearchParameters()
        params.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        params.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        params.time_limit.seconds = 5

        solution = routing.SolveWithParameters(params)
        if not solution:
            self.get_logger().error("TSP solver found no solution")
            raise ValueError("TSP SOLVER ERROR")

        # Walk the route, skipping node 0 (start) and dummy end
        ordered = []
        index = solution.Value(routing.NextVar(routing.Start(0)))  # skip start
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node != dummy_end_idx:
                ordered.append(poses[node - 1])  # -1: node 0 is start
            index = solution.Value(routing.NextVar(index))
        return ordered
        



def print_menu():
    print("\n=== ARPA Core Control ===")
    print("1. Go home")
    print("2. Remove collision plane")
    print("3. Trigger behavior")
    print("4. Update depth")
    print("5. Motor control")
    print("6. Send toolhead home")
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
                node.go_home(toolhead=False)

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

            elif choice == "6":
                node.go_home(toolhead=True)

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
