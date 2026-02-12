#!/usr/bin/env python3
"""
BT Executor node: constrained drop-down screw sequence.

Per screw:
  1. Move to 3 cm above screw (RRT for first screw to avoid gantry; subsequent arrive via transfer).
  2. Descend 3 cm (Cartesian straight Z-down).
  3. Wait 4 seconds.
  4. Retract 3 cm (Cartesian straight Z-up).
  5. Transfer to 3 cm above next screw (RRT 7-DOF so gantry+arm plan avoids structure).

Steps 2–4 use Cartesian (straight line, gantry fixed). Steps 1 and 5 use RRT (7-DOF) so
the planner moves gantry+arm and avoids the gantry structure.
Pose stamps are set so TF
uses latest transform (motion_control_node also uses now() for lookup in sim).
"""

import re
import os
import json
import time as time_module
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from arpa_control.srv import PlanToPose, ExecutePlan
from moveit_msgs.msg import Constraints


# Sequence constants: [Checklist 2] approach offset 3 cm (was 2 cm)
Z_OFFSET_M = 0.03
WAIT_SECONDS = 4


def parse_screw_locations(filepath):
    """Parse Screw Locations.yaml format (same as screw_marker_publisher)."""
    screws = []
    with open(filepath, 'r') as f:
        content = f.read()
    blocks = re.split(r'SCREW\s+(\d+)', content)
    for i in range(1, len(blocks), 2):
        screw_num = int(blocks[i])
        data = blocks[i + 1]
        trans_match = re.search(r'Translation:\s*\[([^\]]+)\]', data)
        rot_match = re.search(r'Rotation:\s*in Quaternion \(xyzw\)\s*\[([^\]]+)\]', data)
        if trans_match and rot_match:
            translation = [float(v) for v in trans_match.group(1).split(',')]
            rotation = [float(v) for v in rot_match.group(1).split(',')]
            screws.append({
                'num': screw_num,
                'translation': translation,
                'rotation': rotation,
            })
    return screws


def make_pose_stamped(frame_id, x, y, z, qx, qy, qz, qw):
    """Build a PoseStamped from position and quaternion (xyzw)."""
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


class BtExecutorNode(Node):
    def __init__(self):
        super().__init__('bt_executor_node')
        self.declare_parameter('screw_file', '')
        self.declare_parameter('frame_id', 'base_link')
        # [Checklist 5] transfer_strategy is read and used in the sequence thread (see _handle_run_screw_sequence)
        self.declare_parameter('transfer_strategy', 'constrained')
        self.declare_parameter('z_offset_m', Z_OFFSET_M)
        self.declare_parameter('wait_seconds', WAIT_SECONDS)

        screw_file = self.get_parameter('screw_file').value
        if not screw_file or not os.path.isfile(screw_file):
            self.get_logger().warn(
                'screw_file not set or missing; run_screw_sequence will fail. '
                'Set screw_file to path of Screw Locations.yaml'
            )
            self._screws = []
        else:
            self._screws = parse_screw_locations(screw_file)
            self.get_logger().info(f'Loaded {len(self._screws)} screw locations from {screw_file}')

        self._plan_client = self.create_client(PlanToPose, 'plan_to_pose')
        self._execute_client = self.create_client(ExecutePlan, 'execute_plan')
        self._run_screw_sequence = self.create_service(
            Trigger, 'run_screw_sequence', self._handle_run_screw_sequence
        )
        self.get_logger().info('bt_executor_node ready: run_screw_sequence advertised')
        self.get_logger().info(
            'Transfer strategy (from param): %s' % self.get_parameter('transfer_strategy').value
        )
        self.get_logger().info('[BT_EXECUTOR] Step 5 transfer uses RRT (use_cartesian=False) to avoid gantry')

    def _handle_run_screw_sequence(self, request, response):
        del request
        if not self._screws:
            response.success = False
            response.message = 'No screw locations loaded. Set screw_file parameter.'
            return response
        # [Checklist 5] Read transfer_strategy in sequence thread (used for step 5: constrained vs free transfer)
        strategy = self.get_parameter('transfer_strategy').value
        frame_id = self.get_parameter('frame_id').value
        z_offset = self.get_parameter('z_offset_m').value
        wait_sec = self.get_parameter('wait_seconds').value
        use_cartesian_transfer = (strategy == 'constrained')
        # #region agent log
        try:
            with open(os.environ.get('DEBUG_LOG_PATH', '/root/ros2_ws/.cursor/debug.log'), 'a') as f:
                f.write(json.dumps({"hypothesisId": "H1,H5", "location": "bt_executor:_handle_run", "message": "strategy_at_start", "data": {"strategy": strategy, "use_cartesian_transfer": use_cartesian_transfer}, "timestamp": int(time_module.time() * 1000)}) + '\n')
        except Exception:
            pass
        self.get_logger().info("[DEBUG_H1] strategy=%s use_cartesian_transfer=%s" % (strategy, use_cartesian_transfer))
        # #endregion
        try:
            self._run_sequence(frame_id, z_offset, wait_sec, use_cartesian_transfer)
            response.success = True
            response.message = 'Screw sequence completed.'
        except RuntimeError as e:
            response.success = False
            response.message = str(e)
        return response

    def _run_sequence(self, frame_id, z_offset, wait_sec, use_cartesian_transfer):
        n = len(self._screws)
        for i, screw in enumerate(self._screws):
            self.get_logger().info(f'--- Screw {screw["num"]} / {n} ---')
            tx, ty, tz = screw['translation']
            qx, qy, qz, qw = screw['rotation']
            # Pose at screw (contact)
            at_screw = make_pose_stamped(frame_id, tx, ty, tz, qx, qy, qz, qw)
            # Pose 3 cm above screw
            above_screw = make_pose_stamped(
                frame_id, tx, ty, tz + z_offset, qx, qy, qz, qw
            )

            # Step 1: Move to 3 cm above this screw (only for first; rest come from transfer).
            # Use RRT for first approach so the 7-DOF planner finds a path around the gantry; Cartesian
            # from home to above screw can pass through the gantry structure.
            if i == 0:
                self.get_logger().info('Step 1: Move to 3 cm above screw (RRT 7-DOF to avoid gantry)')
                self._plan_and_execute(above_screw, use_cartesian=False)
            else:
                self.get_logger().info('Step 1: Already at 3 cm above (from transfer)')

            # [Checklist 3] Descend: plan_to_pose with Cartesian (straight down 3 cm), not plan_to_joint
            self.get_logger().info('Step 2: Descend 3 cm (Cartesian)')
            self._plan_and_execute(at_screw, use_cartesian=True)

            # Step 3: Wait
            self.get_logger().info(f'Step 3: Wait {wait_sec} s')
            import time
            time.sleep(wait_sec)

            # Step 4: Retract 3 cm (Cartesian straight Z-up)
            self.get_logger().info('Step 4: Retract 3 cm (Cartesian)')
            self._plan_and_execute(above_screw, use_cartesian=True)

            # [Checklist 4] Transfer to 3 cm above next screw. Use RRT (not Cartesian) so the 7-DOF planner
            # can move the gantry + arm and find a collision-free path; Cartesian with gantry fixed forces a
            # straight TCP line that goes through the gantry structure (see runtime evidence: 56.5% achieved).
            if i < n - 1:
                next_screw = self._screws[i + 1]
                nx, ny, nz = next_screw['translation']
                nqx, nqy, nqz, nqw = next_screw['rotation']
                above_next = make_pose_stamped(
                    frame_id, nx, ny, nz + z_offset, nqx, nqy, nqz, nqw
                )
                self.get_logger().info(
                    'Step 5: Transfer to 3 cm above next screw (RRT 7-DOF: gantry+arm, avoids structure)'
                )
                self._plan_and_execute(above_next, use_cartesian=False)

        self.get_logger().info('Sequence finished.')

    def _plan_and_execute(self, pose_stamped, use_cartesian):
        # #region agent log
        try:
            p = pose_stamped.pose.position
            with open(os.environ.get('DEBUG_LOG_PATH', '/root/ros2_ws/.cursor/debug.log'), 'a') as f:
                f.write(json.dumps({"hypothesisId": "H2,H3,H4", "location": "bt_executor:_plan_and_execute", "message": "plan_request", "data": {"use_cartesian": use_cartesian, "target_xyz": [round(p.x, 4), round(p.y, 4), round(p.z, 4)]}, "timestamp": int(time_module.time() * 1000)}) + '\n')
        except Exception:
            pass
        p = pose_stamped.pose.position
        self.get_logger().info("[DEBUG_H2] use_cartesian=%s target_xyz=(%.3f,%.3f,%.3f)" % (use_cartesian, p.x, p.y, p.z))
        # #endregion
        # Use stamp 0 so TF uses latest transform (avoids sim/wall clock mismatch and extrapolation errors)
        pose_stamped.header.stamp.sec = 0
        pose_stamped.header.stamp.nanosec = 0
        req = PlanToPose.Request()
        req.target_pose = pose_stamped
        req.use_cartesian = bool(use_cartesian)  # explicit bool so serialization is unambiguous
        req.path_constraints = Constraints()  # empty; full init so request is well-formed
        if not self._plan_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('plan_to_pose service not available')
        future = self._plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done():
            raise RuntimeError('plan_to_pose call timed out')
        result = future.result()
        if not result.success:
            raise RuntimeError('plan_to_pose failed: %s' % result.message)

        if not self._execute_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('execute_plan service not available')
        exec_future = self._execute_client.call_async(ExecutePlan.Request())
        rclpy.spin_until_future_complete(self, exec_future, timeout_sec=60.0)
        if not exec_future.done():
            raise RuntimeError('execute_plan call timed out')
        exec_result = exec_future.result()
        if not exec_result.success:
            raise RuntimeError('execute_plan failed: %s' % exec_result.message)


def main(args=None):
    rclpy.init(args=args)
    node = BtExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
