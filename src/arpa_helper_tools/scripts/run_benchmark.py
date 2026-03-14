#!/usr/bin/env python3
"""
Cost-weight ablation benchmark: run 1000 random goal poses through 8 cost
weight configurations to study each IK cost component's effect on planning.

Assumes sim + motion_control are already running:
  ros2 launch arpa_bringup arpa_sim.launch.py use_corridor_constraint:=true

Usage:
  ros2 run arpa_helper_tools run_benchmark.py \
      --poses scan_results/randomzgoals.csv \
      --output-dir scan_results

For a single case:
  ros2 run arpa_helper_tools run_benchmark.py \
      --poses scan_results/randomzgoals.csv \
      --case baseline
"""

import argparse
import csv
import os
import subprocess
import sys
import threading
import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core_functionality_node import CoreNode


MOTION_CONTROL_NODE = '/motion_control_node'

# (name, cost_w_joint, cost_w_proximity, cost_w_area)
COST_CASES = [
    ("baseline",       0.0, 0.0, 0.0),  # Case 0: no cost preference
    ("all_equal",      1.0, 1.0, 1.0),  # Case 1: all three equal
    ("no_joint",       0.0, 1.0, 1.0),  # Case 2: ablate joint
    ("no_proximity",   1.0, 0.0, 1.0),  # Case 3: ablate proximity
    ("no_area",        1.0, 1.0, 0.0),  # Case 4: ablate area
    ("joint_only",     1.0, 0.0, 0.0),  # Case 5: solo joint
    ("proximity_only", 0.0, 1.0, 0.0),  # Case 6: solo proximity
    ("area_only",      0.0, 0.0, 1.0),  # Case 7: solo area
]

COST_CASE_NAMES = [c[0] for c in COST_CASES]


class CostBenchmark(Node):
    """Benchmark node for cost-weight ablation study."""

    def __init__(self):
        super().__init__('cost_benchmark_node')
        self.core_node = CoreNode()
        self._tf_buffer = self.core_node.tf_buffer
        time.sleep(2.0)
        self._base_frame = self._resolve_base_frame()
        self.get_logger().info(f"Using base frame: {self._base_frame}")

        # Path sampling for actual EE path length during execution
        self._path_sample_points: List[Tuple[float, float, float]] = []
        self._path_sample_stop = threading.Event()
        self._path_sample_interval_s = 0.02  # 50 Hz

    def _resolve_base_frame(self) -> str:
        for frame in ('floor_link', 'world'):
            try:
                if self._tf_buffer.can_transform(
                    frame, 'tool0', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0)
                ):
                    return frame
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
        self.get_logger().warn("Could not detect floor_link or world; defaulting to world")
        return 'world'

    def get_current_ee_pose(self) -> PoseStamped:
        trans = self._tf_buffer.lookup_transform(
            self._base_frame, 'tool0',
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        )
        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = trans.transform.translation.z
        pose.pose.orientation = trans.transform.rotation
        return pose

    def _path_sampler_thread_fn(self) -> None:
        while not self._path_sample_stop.wait(timeout=self._path_sample_interval_s):
            try:
                pose = self.get_current_ee_pose()
                p = pose.pose.position
                self._path_sample_points.append((p.x, p.y, p.z))
            except Exception:
                pass

    def _compute_path_length_m(self, points: List[Tuple[float, float, float]]) -> float:
        if len(points) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(points)):
            a, b = points[i - 1], points[i]
            total += np.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2 + (b[2]-a[2])**2)
        return total

    def load_poses_from_csv(self, csv_file: str) -> List[Dict]:
        poses = []
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                poses.append({
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'z': float(row['z']),
                    'qx': float(row['qx']),
                    'qy': float(row['qy']),
                    'qz': float(row['qz']),
                    'qw': float(row['qw']),
                })
        self.get_logger().info(f"Loaded {len(poses)} poses from {csv_file}")
        return poses

    def run_case(self, case_name: str, goal_poses: List[Dict],
                 output_dir: str) -> None:
        """Run all goal poses for a single cost-weight case, write CSVs."""

        os.makedirs(output_dir, exist_ok=True)
        n_poses = len(goal_poses)

        self.get_logger().info(f"\n{'='*80}")
        self.get_logger().info(f"Starting cost case: {case_name} ({n_poses} poses)")
        self.get_logger().info(f"{'='*80}\n")

        run_start = time.time()
        successful = 0
        detail_records = []

        for i, goal in enumerate(goal_poses):
            # Get current pose
            try:
                current_pose = self.get_current_ee_pose()
            except Exception as e:
                self.get_logger().error(f"Pose {i+1}: Failed to get current pose: {e}")
                detail_records.append(self._empty_detail(case_name, i+1, goal))
                continue

            dx = goal['x'] - current_pose.pose.position.x
            dy = goal['y'] - current_pose.pose.position.y
            dz = goal['z'] - current_pose.pose.position.z
            cart_dist = np.sqrt(dx**2 + dy**2 + dz**2)

            # Plan
            manipulability = 0.0
            num_ik_solutions = 0
            selected_ik_solution_index = -1
            plan_start = time.time()
            try:
                plan_ok, manipulability = self.core_node.plan_to_pose(
                    goal['x'], goal['y'], goal['z'],
                    goal['qx'], goal['qy'], goal['qz'], goal['qw'],
                    frame_id=self._base_frame
                )
                num_ik_solutions = self.core_node.last_plan_num_ik_solutions
                selected_ik_solution_index = self.core_node.last_plan_selected_ik_solution_index
            except Exception as e:
                self.get_logger().error(f"Pose {i+1}: plan exception: {e}")
                plan_ok = False
            plan_time = time.time() - plan_start

            # Execute
            exec_time = 0.0
            path_length = 0.0
            success = plan_ok
            if plan_ok:
                self._path_sample_points.clear()
                self._path_sample_stop.clear()
                sampler = threading.Thread(target=self._path_sampler_thread_fn, daemon=True)
                sampler.start()
                exec_start = time.time()
                try:
                    exec_ok = self.core_node.execute_plan()
                    if not exec_ok:
                        success = False
                except Exception as e:
                    self.get_logger().error(f"Pose {i+1}: execute exception: {e}")
                    success = False
                finally:
                    self._path_sample_stop.set()
                    sampler.join(timeout=1.0)
                path_length = self._compute_path_length_m(self._path_sample_points)
                exec_time = time.time() - exec_start

            if success:
                successful += 1

            detail_records.append({
                'case': case_name,
                'pose_index': i + 1,
                'goal_x': goal['x'],
                'goal_y': goal['y'],
                'goal_z': goal['z'],
                'cartesian_distance_m': cart_dist,
                'path_length_m': path_length,
                'plan_time_s': plan_time,
                'execution_time_s': exec_time,
                'success': int(success),
                'manipulability_score': manipulability,
                'num_ik_solutions': num_ik_solutions,
                'selected_ik_solution_index': selected_ik_solution_index,
            })

            status = "OK" if success else "FAIL"
            self.get_logger().info(
                f"[{case_name}] {i+1:4d}/{n_poses}: {status} | "
                f"Plan: {plan_time:.3f}s | Exec: {exec_time:.3f}s | "
                f"Dist: {cart_dist:.3f}m | Path: {path_length:.3f}m | "
                f"Manip: {manipulability:.6f} | IKs: {num_ik_solutions} | "
                f"Chosen: {selected_ik_solution_index}"
            )

        wall_time = time.time() - run_start
        rate = successful / n_poses if n_poses else 0.0

        self.get_logger().info(f"\n{'='*80}")
        self.get_logger().info(f"Case {case_name} done: {successful}/{n_poses} ({rate:.1%}) in {wall_time:.1f}s")
        self.get_logger().info(f"{'='*80}\n")

        # Write run-level summary CSV
        summary_path = os.path.join(output_dir, f"benchmark_cost_{case_name}.csv")
        with open(summary_path, 'w', newline='') as f:
            w = csv.writer(f)
            # Compute avg manipulability over successful poses only
            manip_scores = [r['manipulability_score'] for r in detail_records if r['success']]
            avg_manip = sum(manip_scores) / len(manip_scores) if manip_scores else 0.0
            successful_records = [r for r in detail_records if r['success']]
            avg_num_ik = (
                sum(r['num_ik_solutions'] for r in successful_records) / len(successful_records)
                if successful_records else 0.0
            )
            w.writerow(['case', 'completed', 'total', 'success_rate', 'wall_s',
                         'sum_plan_time_s', 'sum_execution_time_s', 'sum_path_length_m',
                         'avg_manipulability', 'avg_num_ik_solutions'])
            w.writerow([
                case_name,
                successful,
                n_poses,
                f"{rate:.4f}",
                f"{wall_time:.2f}",
                f"{sum(r['plan_time_s'] for r in detail_records):.2f}",
                f"{sum(r['execution_time_s'] for r in detail_records):.2f}",
                f"{sum(r['path_length_m'] for r in detail_records):.3f}",
                f"{avg_manip:.6f}",
                f"{avg_num_ik:.3f}",
            ])
        self.get_logger().info(f"Wrote {summary_path}")

        # Write detail CSV
        detail_path = os.path.join(output_dir, f"benchmark_cost_{case_name}_detail.csv")
        with open(detail_path, 'w', newline='') as f:
            fieldnames = ['case', 'pose_index', 'goal_x', 'goal_y', 'goal_z',
                          'cartesian_distance_m', 'path_length_m',
                          'plan_time_s', 'execution_time_s', 'success',
                          'manipulability_score', 'num_ik_solutions',
                          'selected_ik_solution_index']
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(detail_records)
        self.get_logger().info(f"Wrote {detail_path}")

    def _empty_detail(self, case_name, index, goal):
        return {
            'case': case_name,
            'pose_index': index,
            'goal_x': goal['x'],
            'goal_y': goal['y'],
            'goal_z': goal['z'],
            'cartesian_distance_m': 0.0,
            'path_length_m': 0.0,
            'plan_time_s': 0.0,
            'execution_time_s': 0.0,
            'success': 0,
            'manipulability_score': 0.0,
            'num_ik_solutions': 0,
            'selected_ik_solution_index': -1,
        }


def configure_planner(planning_time: float, use_analytical_ik: bool = True) -> bool:
    """Set planner params on motion_control_node before running the benchmark."""
    params = [
        ('planning_time', str(planning_time)),
        ('use_analytical_ik', 'true' if use_analytical_ik else 'false'),
    ]
    print(
        f"\n--- Configuring planner: planning_time={planning_time}, "
        f"use_analytical_ik={use_analytical_ik} ---"
    )
    for name, value in params:
        cmd = ['ros2', 'param', 'set', MOTION_CONTROL_NODE, name, value]
        print(f"  {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode != 0:
            print(f"  ERROR: {result.stderr.strip()}", file=sys.stderr)
            return False
    print("--- Planner configured ---\n")
    return True


def set_cost_weights(w_joint: float, w_proximity: float, w_area: float) -> bool:
    """Set cost weight params on motion_control_node via ros2 param set."""
    params = [
        ('cost_w_joint', str(w_joint)),
        ('cost_w_proximity', str(w_proximity)),
        ('cost_w_area', str(w_area)),
    ]
    print(f"\n--- Setting cost weights: joint={w_joint}, proximity={w_proximity}, area={w_area} ---")
    for name, value in params:
        cmd = ['ros2', 'param', 'set', MOTION_CONTROL_NODE, name, value]
        print(f"  {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode != 0:
            print(f"  ERROR: {result.stderr.strip()}", file=sys.stderr)
            return False
    print("--- Weights set ---\n")
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Cost-weight ablation benchmark (8 cases x N poses)'
    )
    parser.add_argument(
        '--poses', type=str, default='scan_results/randomzgoals.csv',
        help='Path to CSV file with goal poses (x,y,z,qx,qy,qz,qw)',
    )
    parser.add_argument(
        '--output-dir', type=str, default='scan_results',
        help='Directory for output CSV files (default: scan_results)',
    )
    parser.add_argument(
        '--case', type=str, choices=COST_CASE_NAMES, default=None,
        help='Run only a single case (default: run all 8)',
    )
    parser.add_argument(
        '--planning-time', type=float, default=20.0,
        help='Planning time to set on motion_control_node before the benchmark',
    )
    args = parser.parse_args()

    # Determine which cases to run
    if args.case:
        cases = [c for c in COST_CASES if c[0] == args.case]
    else:
        cases = COST_CASES

    # Resolve poses path
    poses_path = args.poses
    if not os.path.isabs(poses_path) and not os.path.exists(poses_path):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        fallback = os.path.abspath(os.path.join(
            script_dir, '..', '..', 'scan_results', os.path.basename(poses_path)
        ))
        if os.path.exists(fallback):
            poses_path = fallback

    rclpy.init(args=sys.argv)

    try:
        benchmark = CostBenchmark()

        if not os.path.exists(poses_path):
            benchmark.get_logger().error(f"Poses file not found: {poses_path}")
            return

        goal_poses = benchmark.load_poses_from_csv(poses_path)

        if not configure_planner(args.planning_time, use_analytical_ik=True):
            benchmark.get_logger().error("Failed to configure planning_time/use_analytical_ik")
            return

        for case_name, w_joint, w_prox, w_area in cases:
            if not set_cost_weights(w_joint, w_prox, w_area):
                benchmark.get_logger().error(f"Failed to set weights for {case_name}, skipping")
                continue

            benchmark.run_case(
                case_name=case_name,
                goal_poses=goal_poses,
                output_dir=os.path.abspath(args.output_dir),
            )

        benchmark.get_logger().info("All cases complete!")

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
