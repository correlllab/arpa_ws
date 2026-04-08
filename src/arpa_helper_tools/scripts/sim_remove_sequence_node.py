#!/usr/bin/env python3
"""
Sim Remove Sequence node: iterates all parts in the parts list,
planning hover -> descend -> wait -> retract for each part.
Uses only MoveIt planning services (no hardware calls).
"""

import os
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from arpa_control.srv import PlanToPose, ExecutePlan
from moveit_msgs.msg import Constraints
from ament_index_python.packages import get_package_share_directory

Z_HEIGHT = 0.91
Z_OFFSET_M = 0.03
WAIT_SECONDS = 4
FRAME_ID = "floor_link"
SERVICE_WAIT_SEC = 10.0
PLAN_TIMEOUT_SEC = 60.0
EXEC_TIMEOUT_SEC = 60.0


def compute_tool_down_quaternion(x, y):
    """
    Compute quaternion for tool-pointing-down orientation,
    same logic as _remove_part_cb in core_functionality_node.py.
    z_hat = [0, 0, -1], y_hat = normalize([-x, -y, 0]), x_hat = cross(y, z).
    Returns (qx, qy, qz, qw).
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    z_hat = np.array([0.0, 0.0, -1.0])
    toward_origin = np.array([-x, -y, 0.0])
    norm = np.linalg.norm(toward_origin)
    y_hat = toward_origin / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0])
    x_hat = np.cross(y_hat, z_hat)
    R = np.column_stack([x_hat, y_hat, z_hat])
    qx, qy, qz, qw = Rotation.from_matrix(R).as_quat()
    return float(qx), float(qy), float(qz), float(qw)


def make_pose_stamped(frame_id, x, y, z, qx, qy, qz, qw):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp.sec = 0
    ps.header.stamp.nanosec = 0
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


class SimRemoveSequenceNode(Node):
    def __init__(self):
        super().__init__('sim_remove_sequence_node')

        self._cb_group = ReentrantCallbackGroup()

        self._plan_client = self.create_client(
            PlanToPose, 'plan_to_pose', callback_group=self._cb_group
        )
        self._exec_client = self.create_client(
            ExecutePlan, 'execute_plan', callback_group=self._cb_group
        )
        self._progress_pub = self.create_publisher(String, '/triggered_behavior', 10)

        self.create_service(
            Trigger, 'run_sim_remove_sequence',
            self._handle_run, callback_group=self._cb_group
        )

        self._parts = self._load_parts()
        self.get_logger().info(
            f'sim_remove_sequence_node ready: {len(self._parts)} parts loaded. '
            'Call run_sim_remove_sequence to start.'
        )

    def _load_parts(self):
        try:
            pkg_share = get_package_share_directory("arpa_helper_tools")
        except Exception:
            pkg_share = os.path.join(os.path.dirname(__file__), "..", "resources")
        json_path = os.path.join(
            pkg_share, "resources", "parts_lists", "hyundai_ioniq_parts_list.json"
        )
        if not os.path.isfile(json_path):
            self.get_logger().error(f"Parts list not found at {json_path}")
            return {}
        with open(json_path) as f:
            return json.load(f)

    def _publish_progress(self, msg):
        self._progress_pub.publish(String(data=msg))
        self.get_logger().info(msg)

    def _plan_and_execute(self, pose_stamped, use_cartesian=False):
        req = PlanToPose.Request()
        req.target_pose = pose_stamped
        req.use_cartesian = bool(use_cartesian)
        req.path_constraints = Constraints()

        if not self._plan_client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            raise RuntimeError(f'plan_to_pose service not available after {SERVICE_WAIT_SEC:.0f}s')

        self.get_logger().info(f'  -> calling plan_to_pose (cartesian={use_cartesian})')
        future = self._plan_client.call_async(req)
        start = time.time()
        last_log = start
        while rclpy.ok() and not future.done():
            now = time.time()
            if now - start > PLAN_TIMEOUT_SEC:
                raise RuntimeError(f'plan_to_pose timed out after {PLAN_TIMEOUT_SEC:.0f}s')
            if now - last_log >= 5.0:
                self.get_logger().warn(f'  -> still waiting for plan_to_pose ({now - start:.0f}s)...')
                last_log = now
            time.sleep(0.05)

        if not future.done():
            raise RuntimeError('plan_to_pose did not complete (shutdown?)')

        result = future.result()
        self.get_logger().info(f'  -> plan_to_pose result: success={result.success}')
        if not result.success:
            raise RuntimeError(f'plan_to_pose failed: {result.message}')

        if not self._exec_client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            raise RuntimeError(f'execute_plan service not available after {SERVICE_WAIT_SEC:.0f}s')

        self.get_logger().info('  -> calling execute_plan')
        exec_future = self._exec_client.call_async(ExecutePlan.Request())
        start = time.time()
        last_log = start
        while rclpy.ok() and not exec_future.done():
            now = time.time()
            if now - start > EXEC_TIMEOUT_SEC:
                raise RuntimeError(f'execute_plan timed out after {EXEC_TIMEOUT_SEC:.0f}s')
            if now - last_log >= 5.0:
                self.get_logger().warn(f'  -> still waiting for execute_plan ({now - start:.0f}s)...')
                last_log = now
            time.sleep(0.05)

        if not exec_future.done():
            raise RuntimeError('execute_plan did not complete (shutdown?)')

        exec_result = exec_future.result()
        if not exec_result.success:
            raise RuntimeError(f'execute_plan failed: {exec_result.message}')

    def _handle_run(self, request, response):
        del request
        if not self._parts:
            response.success = False
            response.message = 'No parts loaded'
            return response

        if not self._plan_client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            response.success = False
            response.message = f'plan_to_pose service not available (waited {SERVICE_WAIT_SEC:.0f}s)'
            return response

        n = len(self._parts)
        failed = []

        for i, (name, coords) in enumerate(self._parts.items()):
            part_num = i + 1
            x, y, z_raw = coords
            qx, qy, qz, qw = compute_tool_down_quaternion(x, y)

            hover_z = Z_HEIGHT + Z_OFFSET_M
            at_z = Z_HEIGHT

            hover_pose = make_pose_stamped(FRAME_ID, x, y, hover_z, qx, qy, qz, qw)
            at_pose = make_pose_stamped(FRAME_ID, x, y, at_z, qx, qy, qz, qw)

            try:
                # Step 1: Plan to hover position (free-space)
                self._publish_progress(
                    f'Sim Remove: Part {part_num}/{n} ({name}) — hovering'
                )
                self._plan_and_execute(hover_pose, use_cartesian=False)

                # Step 2: Descend to part (Cartesian)
                self._publish_progress(
                    f'Sim Remove: Part {part_num}/{n} ({name}) — descending'
                )
                self._plan_and_execute(at_pose, use_cartesian=True)

                # Step 3: Wait (simulate unscrewing)
                self._publish_progress(
                    f'Sim Remove: Part {part_num}/{n} ({name}) — unscrewing ({WAIT_SECONDS}s)'
                )
                time.sleep(WAIT_SECONDS)

                # Step 4: Retract (Cartesian)
                self._publish_progress(
                    f'Sim Remove: Part {part_num}/{n} ({name}) — retracting'
                )
                self._plan_and_execute(hover_pose, use_cartesian=True)

            except RuntimeError as e:
                self.get_logger().warn(
                    f'Part {part_num}/{n} ({name}) failed: {e} — skipping'
                )
                self._publish_progress(
                    f'Sim Remove: Part {part_num}/{n} ({name}) FAILED — skipping'
                )
                failed.append(name)
                continue

        if failed:
            response.success = True
            response.message = (
                f'Sequence done. {n - len(failed)}/{n} succeeded. '
                f'Failed: {", ".join(failed)}'
            )
        else:
            response.success = True
            response.message = f'Sequence complete. All {n} parts done.'

        self._publish_progress(f'Sim Remove: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimRemoveSequenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
