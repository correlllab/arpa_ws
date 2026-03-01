#!/usr/bin/env python3
"""
Failed Planning Investigator

Provides the /fix_failed_plan service. When a plan fails, this node receives
the failed target pose, reads the current joint states, and tests 100 different
linear actuator positions (0.2–1.9) as seed joint values via /plan_to_pose.
Returns the first successful seed_joint_values.

Publishes MarkerArray to /failed_plan_markers for RViz visualization:
  - Green spheres: actuator positions where planning succeeded
  - Red spheres: actuator positions where planning failed
  - Blue arrow: the target pose that was requested
"""

import itertools
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener
from arpa_control.srv import FixFailedPlan, PlanToPose
from rclpy.logging import get_logger

# Joint order in the ur16e_on_gantry planning group
PLANNING_GROUP_JOINTS = [
    "linear_actuator_to_linear_actuator_plate_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Joint limits from YAML
JOINT_LIMITS = {
    "linear_actuator_to_linear_actuator_plate_joint": (0.2, 1.9),
    "shoulder_pan_joint":  (-2*math.pi, 2*math.pi),
    "shoulder_lift_joint": (-2*math.pi, 2*math.pi),
    "elbow_joint":         (-math.pi,   math.pi),      # artificially limited
    "wrist_1_joint":       (-2*math.pi, 2*math.pi),
    "wrist_2_joint":       (-2*math.pi, 2*math.pi),
    "wrist_3_joint":       (-2*math.pi, 2*math.pi),
}

def normalize_to_limits(angle, min_limit, max_limit):
    """
    Normalize angle to [-pi, pi] first, then check if it falls
    within joint limits. If not, try adding/subtracting 2pi
    to find a valid equivalent within limits.
    """
    # Start with [-pi, pi] normalization
    angle = math.atan2(math.sin(angle), math.cos(angle))
    
    if min_limit <= angle <= max_limit:
        return angle
    
    # Try 2pi shift
    for offset in [2*math.pi, -2*math.pi]:
        shifted = angle + offset
        if min_limit <= shifted <= max_limit:
            return shifted
    
    # Clamp as last resort
    return max(min_limit, min(max_limit, angle))


def compute_actuator_position(current_actuator, current_shoulder_pan, actuator_retract_delta=0.25):
    """
    Adjust actuator position based on shoulder pan angle.
    
    Rule: Plus/minus delta based on shoulder_pan angle (in radians):
    - [-360°, -270°): plus
    - [-270°, -90°): minus
    - [-90°, 90°): plus
    - [90°, 270°): minus
    - [270°, 360°): plus
    """
    # Convert to degrees for easier logic (shoulder_pan is in radians)
    angle_deg = math.degrees(current_shoulder_pan)
    
    # Normalize angle to [-360, 360] range
    while angle_deg > 360:
        angle_deg -= 360
    while angle_deg < -360:
        angle_deg += 360
    
    # Determine if we add or subtract based on angle ranges
    if (-360 <= angle_deg < -270) or (-90 <= angle_deg < 90) or (270 <= angle_deg <= 360):
        # Plus: move actuator in positive direction
        new_actuator = current_actuator + actuator_retract_delta
        direction = "plus"
    else:  # (-270 <= angle_deg < -90) or (90 <= angle_deg < 270)
        # Minus: move actuator in negative direction
        new_actuator = current_actuator - actuator_retract_delta
        direction = "minus"
    
    # Clamp to safe gantry range
    MIN_ACTUATOR = 0.2
    MAX_ACTUATOR = 1.9
    clamped_actuator = max(MIN_ACTUATOR, min(MAX_ACTUATOR, new_actuator))
    
    # Debug logging
    try:
        from rclpy.logging import get_logger
        logger = get_logger('compute_actuator_position')
        logger.info(f"Actuator adjustment: shoulder_pan={angle_deg:.1f}° → {direction} → "
                   f"{current_actuator:.3f} → {clamped_actuator:.3f}m")
    except:
        pass
    
    return clamped_actuator


def get_branch_seeds(current_joint_values, target_x=None, target_y=None, target_z=None):
    curr_actuator = current_joint_values[0]
    curr_sp = current_joint_values[1]
    curr_sl = current_joint_values[2]
    curr_el = current_joint_values[3]
    curr_w1 = current_joint_values[4]
    curr_w2 = current_joint_values[5]
    curr_w3 = current_joint_values[6]

    # Retract the actuator slightly from its current position
    ACTUATOR_RETRACT_DELTA = 0.1  # tune this (meters)
    seed_actuator = compute_actuator_position(curr_actuator, curr_sp, ACTUATOR_RETRACT_DELTA)
    seed_actuator = normalize_to_limits(
        seed_actuator,
        *JOINT_LIMITS["linear_actuator_to_linear_actuator_plate_joint"]
    )

    # Recompute shoulder pan to still point toward the target from the retracted position
    # if target_x is not None and target_y is not None:
    #     dx = target_x - seed_actuator  # shoulder_x == actuator position
    #     dy = target_y - 0.0            # fixed shoulder Y
    #     seed_sp = normalize_to_limits(
    #         math.atan2(dy, dx) - math.pi / 2.0,
    #         *JOINT_LIMITS["shoulder_pan_joint"]
    #     )
    # else:
    #     seed_sp = curr_sp

    return [{
        "linear_actuator_to_linear_actuator_plate_joint": seed_actuator,
        "shoulder_pan_joint":  curr_sp,
        "shoulder_lift_joint": curr_sl,
        "elbow_joint":         curr_el,
        "wrist_1_joint":       curr_w1,
        "wrist_2_joint":       curr_w2,
        "wrist_3_joint":       curr_w3,
    }]


class FailedPlanningInvestigator(Node):
    def __init__(self):
        super().__init__('failed_planning_investigator')

        self._cb_group = ReentrantCallbackGroup()

        # Subscribe to joint states to get current arm configuration
        self._latest_joint_state = None
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        # Client to the motion control planner
        self._plan_client = self.create_client(
            PlanToPose, 'plan_to_pose', callback_group=self._cb_group)
        self.get_logger().info("Waiting for plan_to_pose service...")
        if not self._plan_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("Timed out waiting for plan_to_pose service")

        # Marker publisher for RViz
        self._marker_pub = self.create_publisher(
            MarkerArray, '/failed_plan_markers', 10)

        # Serve fix_failed_plan
        self.create_service(
            FixFailedPlan, 'fix_failed_plan', self._fix_failed_plan_cb,
            callback_group=self._cb_group)

        # TF2 for looking up wrist_3_link / tool0 pose
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info("fix_failed_plan service ready")
        self.get_logger().info("Publishing visualization markers on /failed_plan_markers")

    def _joint_state_cb(self, msg: JointState):
        self._latest_joint_state = msg

    def _get_current_seed(self):
        """Build a 7-element seed from the latest joint states."""
        js = self._latest_joint_state
        if js is None:
            return None

        # Map joint name -> position from the JointState message
        name_to_pos = dict(zip(js.name, js.position))
        seed = []
        for joint_name in PLANNING_GROUP_JOINTS:
            if joint_name not in name_to_pos:
                self.get_logger().error(f"Joint '{joint_name}' not found in /joint_states")
                return None
            seed.append(name_to_pos[joint_name])
        return seed
    
    def _get_current_pose(self):
        """Look up the current tool0 pose in world frame via tf2."""
        try:
            tf = self._tf_buffer.lookup_transform('world', 'tool0', rclpy.time.Time())
            pose = tf.transform
            self.get_logger().info(
                f"  tool0 in world: "
                f"xyz=({pose.translation.x:.4f}, {pose.translation.y:.4f}, {pose.translation.z:.4f}) "
                f"quat=({pose.rotation.x:.4f}, {pose.rotation.y:.4f}, {pose.rotation.z:.4f}, {pose.rotation.w:.4f})")
            return pose
        except Exception as e:
            self.get_logger().warn(f"tf2 lookup world->tool0 failed: {e}")
            return None

    def _fix_failed_plan_cb(self, request, response):
        pose = request.failed_pose
        p = pose.pose.position
        o = pose.pose.orientation
        current_pose = self._get_current_pose()
        self.get_logger().info("=" * 60)
        self.get_logger().info("fix_failed_plan REQUEST RECEIVED")
        if current_pose is not None:
            cp = current_pose.translation
            self.get_logger().info(
                f"  Current EE (tool0): x={cp.x:.4f}  y={cp.y:.4f}  z={cp.z:.4f}")
        else:
            self.get_logger().warn("  Current EE pose: unavailable (tf2 lookup failed)")
        self.get_logger().info(
            f"  Target position:     x={p.x:.4f}  y={p.y:.4f}  z={p.z:.4f}")
        self.get_logger().info(
            f"  Target orientation:  x={o.x:.4f}  y={o.y:.4f}  z={o.z:.4f}  w={o.w:.4f}")
        self.get_logger().info(
            f"  Frame: {pose.header.frame_id or '(empty)'}")
        current_seed = self._get_current_seed()
        if current_seed is None:
            self.get_logger().error("No joint states available -- cannot investigate")
            response.success = False
            response.seed_joint_values = []
            return response

        current_actuator = current_seed[0]
        
        # Generate branch seeds based on current joint configuration and target
        branch_seeds = get_branch_seeds(
            current_seed, 
            target_x=p.x, 
            target_y=p.y, 
            target_z=p.z
        )

        # Convert branch seed dicts to ordered lists
        branch_lists = []
        for b in branch_seeds:
            branch_lists.append([b[j] for j in PLANNING_GROUP_JOINTS])

        # Prepend current joint state as the first seed to try
        all_seeds = [list(current_seed)] + branch_lists
        num_seeds = len(all_seeds)

        self.get_logger().info(
            f"  Testing {num_seeds} seeds (current + {len(branch_seeds)} IK branches)")
        self.get_logger().info("=" * 60)

        self.get_logger().info("Current joint seed values:")
        for name, val in zip(PLANNING_GROUP_JOINTS, current_seed):
            self.get_logger().info(f"  {name}: {val:.4f}")

        self.get_logger().info(f"Generated {len(branch_seeds)} IK branch seeds:")
        for bi, branch in enumerate(branch_seeds[:5]):  # show first 5
            self.get_logger().info(
                f"  Branch {bi}: act={branch[PLANNING_GROUP_JOINTS[0]]:.3f} "
                f"sp={branch['shoulder_pan_joint']:.3f} sl={branch['shoulder_lift_joint']:.3f} "
                f"el={branch['elbow_joint']:.3f} w1={branch['wrist_1_joint']:.3f}")
        if len(branch_seeds) > 5:
            self.get_logger().info(f"  ... and {len(branch_seeds) - 5} more branches")

        # Track ALL results -- select the plan with fewest trajectory points
        # results: (seed_label, success, seed, message, path_length)
        results = []
        best_seed = None
        best_path_length = float('inf')
        best_label = ""
        start_time = time.monotonic()

        for si, seed in enumerate(all_seeds):
            input(f"Press Enter to test seed {si+1}/{num_seeds}...")
            label = "current" if si == 0 else f"branch {si - 1}"

            req = PlanToPose.Request()
            req.target_pose = pose
            req.seed_joint_values = seed

            future = self._plan_client.call_async(req)
            while not future.done():
                time.sleep(0.5)

            result = future.result()
            path_length = result.path_length if result.success else 0
            results.append((label, result.success, seed, result.message, path_length))

            if result.success:
                self.get_logger().info(
                    f"  [{si+1}/{num_seeds}] {label}  SUCCESS: Path Length = {path_length}")
                if path_length < best_path_length:
                    best_path_length = path_length
                    best_seed = list(seed)
                    best_label = label
            else:
                self.get_logger().info(
                    f"  [{si+1}/{num_seeds}] {label}  FAILED  reason: {result.message}")

        input(f"MOVE ON ?")
        elapsed = time.monotonic() - start_time
        successes = [r for r in results if r[1]]
        failures = [r for r in results if not r[1]]

        # Print summary
        self.get_logger().info("=" * 60)
        self.get_logger().info("INVESTIGATION COMPLETE")
        self.get_logger().info(f"  Total time:  {elapsed:.1f}s")
        self.get_logger().info(
            f"  Successes:   {len(successes)}/{num_seeds}")
        self.get_logger().info(
            f"  Failures:    {len(failures)}/{num_seeds}")

        if successes:
            self.get_logger().info("  Successful seeds (path lengths):")
            for lbl, _, seed, _, pl in successes:
                self.get_logger().info(
                    f"    {lbl}: path_length={pl}  seed={[f'{v:.3f}' for v in seed]}")
            self.get_logger().info(
                f"  Selected '{best_label}' with shortest path ({best_path_length} points)")
        else:
            self.get_logger().warn(
                "  No seed produced a valid plan for this pose!")
            reason_counts = {}
            for _, _, _, msg, _ in failures:
                reason_counts[msg] = reason_counts.get(msg, 0) + 1
            self.get_logger().warn("  Failure reason breakdown:")
            for reason, count in sorted(reason_counts.items(), key=lambda x: -x[1]):
                self.get_logger().warn(f"    [{count:3d}x] {reason}")

        self.get_logger().info("=" * 60)

        if best_seed is not None:
            # Re-plan with the best seed so the motion controller has the shortest trajectory loaded
            self.get_logger().info(
                f"  Re-planning with '{best_label}' to load shortest trajectory...")
            req = PlanToPose.Request()
            req.target_pose = pose
            req.seed_joint_values = best_seed
            future = self._plan_client.call_async(req)
            while not future.done():
                time.sleep(0.5)
            replan_result = future.result()
            if replan_result.success:
                self.get_logger().info(
                    f"  Re-plan succeeded: path_length={replan_result.path_length}")
            else:
                self.get_logger().warn(
                    f"  Re-plan failed: {replan_result.message} (using original plan)")

            response.success = True
            response.seed_joint_values = best_seed
        else:
            response.success = False
            response.seed_joint_values = []
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FailedPlanningInvestigator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
