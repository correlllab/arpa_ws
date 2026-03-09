#!/usr/bin/env python3
"""
Run random 1000 point-to-point benchmark for RRT planning.

Loads 1000 goal poses (or generates them), plans and executes moves sequentially,
records metrics (plan time, execution time, success, distance), and outputs
run-level and detail CSV files.

Prerequisites:
    Start the ARPA stack first so plan_to_pose and execute_plan are available:
      ros2 launch arpa_bringup arpa_sim.launch.py   # or arpa_real.launch.py
    Then run this benchmark in another terminal.

Usage:
    ros2 run arpa_helper_tools run_random_p2p_benchmark.py \
        --test-case corridor_full \
        --poses scan_results/random_1000_goals.csv \
        --output-dir scan_results
"""

import argparse
import csv
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs  # Required for tf2 to handle PoseStamped
import numpy as np

# Import the CoreNode for plan/execute functionality
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core_functionality_node import CoreNode


class RandomP2PBenchmark(Node):
    """Benchmark node for random point-to-point planning and execution."""
    
    def __init__(self):
        super().__init__('random_p2p_benchmark_node')
        
        self.core_node = CoreNode()
        # Use CoreNode's TF buffer (it is updated by CoreNode's spin thread)
        self._tf_buffer = self.core_node.tf_buffer
        
        time.sleep(2.0)
        self._base_frame = self._resolve_base_frame()
        self.get_logger().info(f"Using base frame for EE pose: {self._base_frame}")
        # Path sampling for actual path length (EE positions during execution)
        self._path_sample_points: List[Tuple[float, float, float]] = []
        self._path_sample_stop = threading.Event()
        self._path_sample_interval_s = 0.02  # 50 Hz
    
    def _path_sampler_thread_fn(self) -> None:
        """Sample EE position at fixed interval until stop event is set."""
        while not self._path_sample_stop.wait(timeout=self._path_sample_interval_s):
            try:
                pose = self.get_current_ee_pose()
                p = pose.pose.position
                self._path_sample_points.append((p.x, p.y, p.z))
            except Exception:
                pass  # skip failed lookups
    
    def _compute_path_length_m(self, points: List[Tuple[float, float, float]]) -> float:
        """Total length along sampled EE path (sum of segment lengths)."""
        if len(points) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(points)):
            a, b = points[i - 1], points[i]
            total += np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)
        return total
    
    def _resolve_base_frame(self) -> str:
        """Use floor_link if available in TF, otherwise world (e.g. Gazebo sim)."""
        for frame in ('floor_link', 'world'):
            try:
                if self._tf_buffer.can_transform(frame, 'tool0', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)):
                    return frame
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
        self.get_logger().warn("Could not detect floor_link or world in TF; defaulting to world for sim.")
        return 'world'
    
    def get_current_ee_pose(self) -> PoseStamped:
        """
        Get current end-effector pose via TF (uses CoreNode's buffer).
        
        Returns:
            PoseStamped in base frame (floor_link or world)
        """
        try:
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
        except Exception as e:
            self.get_logger().error(f"Failed to get current EE pose: {e}")
            raise
    
    def load_poses_from_csv(self, csv_file: str) -> List[Dict]:
        """
        Load goal poses from CSV file.
        
        CSV format: x,y,z,qx,qy,qz,qw
        
        Args:
            csv_file: Path to CSV file with goal poses
        
        Returns:
            List of pose dictionaries
        """
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
    
    def compute_cartesian_distance(self, pose1: PoseStamped, pose2: Dict) -> float:
        """
        Compute Euclidean distance between two poses (position only).
        
        Args:
            pose1: Start pose (PoseStamped)
            pose2: Goal pose (dict with x, y, z keys)
        
        Returns:
            Cartesian distance in meters
        """
        dx = pose2['x'] - pose1.pose.position.x
        dy = pose2['y'] - pose1.pose.position.y
        dz = pose2['z'] - pose1.pose.position.z
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        return distance
    
    def dict_to_pose_stamped(self, pose_dict: Dict) -> PoseStamped:
        """
        Convert pose dictionary to PoseStamped message.
        
        Args:
            pose_dict: Dictionary with x, y, z, qx, qy, qz, qw keys
        
        Returns:
            PoseStamped message in base frame (floor_link or world)
        """
        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pose_dict['x']
        pose.pose.position.y = pose_dict['y']
        pose.pose.position.z = pose_dict['z']
        pose.pose.orientation.x = pose_dict['qx']
        pose.pose.orientation.y = pose_dict['qy']
        pose.pose.orientation.z = pose_dict['qz']
        pose.pose.orientation.w = pose_dict['qw']
        
        return pose
    
    def run_benchmark(self, test_case: str, goal_poses: List[Dict],
                     output_dir: str = 'scan_results') -> Tuple[Dict, List[Dict]]:
        """
        Run 1000 point-to-point moves and record metrics.
        
        Args:
            test_case: Test case identifier (e.g., 'corridor_full', 'pure_rrt')
            goal_poses: List of goal poses (dicts with x, y, z, qx, qy, qz, qw)
            output_dir: Directory for output CSV files
        
        Returns:
            Tuple of (run_summary, detail_records)
        """
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Run-level metrics
        run_summary = {
            'run_id': f"{test_case}_{int(time.time())}",
            'test_case': test_case,
            'completed': 0,
            'total': len(goal_poses),
            'success_rate': 0.0,
            'total_wall_s': 0.0,
            'distance_travelled_m': 0.0,
            'path_length_travelled_m': 0.0,
            'sum_plan_time_s': 0.0,
            'sum_execution_time_s': 0.0,
        }
        
        detail_records = []
        
        self.get_logger().info(f"\n{'='*80}")
        self.get_logger().info(f"Starting benchmark: {test_case}")
        self.get_logger().info(f"Total moves: {len(goal_poses)}")
        self.get_logger().info(f"{'='*80}\n")
        
        run_start_time = time.time()
        total_distance = 0.0
        successful_moves = 0
        
        for move_index in range(len(goal_poses)):
            # Get current pose
            try:
                current_pose = self.get_current_ee_pose()
            except Exception as e:
                self.get_logger().error(f"Move {move_index + 1}: Failed to get current pose: {e}")
                continue
            
            # Goal pose
            goal_dict = goal_poses[move_index]
            goal_pose = self.dict_to_pose_stamped(goal_dict)
            
            # Compute distance
            distance = self.compute_cartesian_distance(current_pose, goal_dict)
            total_distance += distance
            
            # Plan (plan_to_pose expects x, y, z, qx, qy, qz, qw, frame_id); timeout and exceptions => failure, continue
            plan_start = time.time()
            try:
                success = self.core_node.plan_to_pose(
                    goal_dict['x'], goal_dict['y'], goal_dict['z'],
                    goal_dict['qx'], goal_dict['qy'], goal_dict['qz'], goal_dict['qw'],
                    frame_id=self._base_frame
                )
            except Exception as e:
                self.get_logger().error(f"Move {move_index + 1}: plan_to_pose exception: {e}")
                success = False
            plan_time = time.time() - plan_start

            # Execute (if planning succeeded); timeout and exceptions => failure, continue
            execution_time = 0.0
            path_length_m = 0.0
            if success:
                exec_start = time.time()
                # Sample EE path during execution for actual path length
                self._path_sample_points.clear()
                self._path_sample_stop.clear()
                sampler = threading.Thread(target=self._path_sampler_thread_fn, daemon=True)
                sampler.start()
                try:
                    exec_ok = self.core_node.execute_plan()
                    if not exec_ok:
                        success = False
                except Exception as e:
                    self.get_logger().error(f"Move {move_index + 1}: execute_plan exception: {e}")
                    success = False
                finally:
                    self._path_sample_stop.set()
                    sampler.join(timeout=1.0)
                path_length_m = self._compute_path_length_m(self._path_sample_points)
                execution_time = time.time() - exec_start
                if success:
                    successful_moves += 1
            
            # Record detail
            detail_record = {
                'run_id': run_summary['run_id'],
                'test_case': test_case,
                'move_index': move_index + 1,
                'start_x': current_pose.pose.position.x,
                'start_y': current_pose.pose.position.y,
                'start_z': current_pose.pose.position.z,
                'goal_x': goal_dict['x'],
                'goal_y': goal_dict['y'],
                'goal_z': goal_dict['z'],
                'cartesian_distance_m': distance,
                'path_length_m': path_length_m,
                'plan_time_s': plan_time,
                'execution_time_s': execution_time,
                'success': int(success),
            }
            detail_records.append(detail_record)
            
            # Logging
            status = "✓" if success else "✗"
            self.get_logger().info(
                f"Move {move_index + 1:3d}/{len(goal_poses)}: {status} | "
                f"Plan: {plan_time:.3f}s | Exec: {execution_time:.3f}s | "
                f"Cartesian: {distance:.3f}m | Path: {path_length_m:.3f}m"
            )
        
        run_end_time = time.time()
        total_wall_time = run_end_time - run_start_time
        
        # Update summary
        run_summary['completed'] = successful_moves
        run_summary['success_rate'] = successful_moves / len(goal_poses) if goal_poses else 0.0
        run_summary['total_wall_s'] = total_wall_time
        run_summary['distance_travelled_m'] = total_distance
        run_summary['path_length_travelled_m'] = sum(r['path_length_m'] for r in detail_records)
        run_summary['sum_plan_time_s'] = sum(r['plan_time_s'] for r in detail_records)
        run_summary['sum_execution_time_s'] = sum(r['execution_time_s'] for r in detail_records)
        
        self.get_logger().info(f"\n{'='*80}")
        self.get_logger().info(f"Benchmark complete: {test_case}")
        self.get_logger().info(f"Success rate: {run_summary['success_rate']:.1%} ({successful_moves}/{len(goal_poses)})")
        self.get_logger().info(f"Total time: {total_wall_time:.2f}s")
        self.get_logger().info(f"Cartesian distance (sum start->goal): {total_distance:.2f}m")
        self.get_logger().info(f"Path length (actual EE travel): {run_summary['path_length_travelled_m']:.2f}m")
        self.get_logger().info(f"{'='*80}\n")
        
        return run_summary, detail_records
    
    def write_csv_files(self, output_dir: str, test_case: str,
                       run_summary: Dict, detail_records: List[Dict]) -> None:
        """
        Write run-level and detail CSV files.
        
        Args:
            output_dir: Directory for output files
            test_case: Test case identifier
            run_summary: Run-level summary dictionary
            detail_records: List of per-move detail records
        """
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Small CSV (run-level)
        small_csv = os.path.join(output_dir, f"benchmark_random1000_{test_case}.csv")
        with open(small_csv, 'w', newline='') as f:
            fieldnames = ['run_id', 'test_case', 'completed', 'total', 'success_rate',
                         'total_wall_s', 'distance_travelled_m', 'path_length_travelled_m',
                         'sum_plan_time_s', 'sum_execution_time_s']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow(run_summary)
        
        self.get_logger().info(f"Wrote run-level summary: {small_csv}")
        
        # Detail CSV (per-move)
        detail_csv = os.path.join(output_dir, f"benchmark_random1000_{test_case}_detail.csv")
        with open(detail_csv, 'w', newline='') as f:
            fieldnames = ['run_id', 'test_case', 'move_index', 'start_x', 'start_y', 'start_z',
                         'goal_x', 'goal_y', 'goal_z', 'cartesian_distance_m', 'path_length_m',
                         'plan_time_s', 'execution_time_s', 'success']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(detail_records)
        
        self.get_logger().info(f"Wrote per-move details: {detail_csv}")


MOTION_CONTROL_NODE = '/motion_control_node'

# Parameter configurations for each test case (planning_time in seconds; can override per case)
TEST_CASE_PARAMS = {
    'pure_rrt': {
        'use_corridor_constraint': 'false',
        'constrain_corridor_position': 'false',
        'constrain_corridor_orientation': 'false',
        'planning_time': '20.0',
    },
    'corridor_full': {
        'use_corridor_constraint': 'true',
        'constrain_corridor_position': 'true',
        'constrain_corridor_orientation': 'true',
        'planning_time': '20.0',
    },
    'corridor_position_only': {
        'use_corridor_constraint': 'true',
        'constrain_corridor_position': 'true',
        'constrain_corridor_orientation': 'false',
        'planning_time': '20.0',
    },
    'corridor_orientation_only': {
        'use_corridor_constraint': 'true',
        'constrain_corridor_position': 'false',
        'constrain_corridor_orientation': 'true',
        'planning_time': '20.0',
    },
}

ALL_TEST_CASES = ['pure_rrt', 'corridor_full', 'corridor_position_only', 'corridor_orientation_only']


def configure_params_for_test_case(test_case: str) -> bool:
    """Set motion_control_node parameters for the given test case via ros2 param set."""
    params = TEST_CASE_PARAMS[test_case]
    print(f"\n--- Configuring parameters for test case: {test_case} ---")
    for param_name, param_value in params.items():
        cmd = ['ros2', 'param', 'set', MOTION_CONTROL_NODE, param_name, param_value]
        print(f"  {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode != 0:
            print(f"  ERROR: Failed to set {param_name}={param_value}: {result.stderr.strip()}")
            return False
    print(f"--- Parameters configured for {test_case} ---\n")
    return True


def main():
    parser = argparse.ArgumentParser(description='Run random 1000 P2P benchmark for RRT planning')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--test-case', type=str,
                       choices=ALL_TEST_CASES,
                       help='Test case identifier')
    group.add_argument('--all', action='store_true',
                       help='Run all 4 test cases sequentially (pure_rrt, corridor_full, corridor_position_only, corridor_orientation_only)')
    parser.add_argument('--poses', type=str, default='scan_results/random_1000_goals.csv',
                       help='Path to CSV file with goal poses')
    parser.add_argument('--output-dir', type=str, default='scan_results',
                       help='Directory for output CSV files')
    parser.add_argument('--seed', type=int, default=42,
                       help='Random seed for reproducibility (if generating poses)')

    args = parser.parse_args()

    test_cases = ALL_TEST_CASES if args.all else [args.test_case]

    # Resolve poses path: if not found relative to CWD, try same path relative to script (where generator writes)
    poses_path = args.poses
    if not os.path.isabs(poses_path) and not os.path.exists(poses_path):
        _script_dir = os.path.dirname(os.path.abspath(__file__))
        _fallback = os.path.abspath(os.path.join(_script_dir, '..', '..', 'scan_results', os.path.basename(poses_path)))
        if os.path.exists(_fallback):
            poses_path = _fallback

    # Initialize ROS2
    rclpy.init(args=sys.argv)

    try:
        # Create benchmark node
        benchmark = RandomP2PBenchmark()

        # Load goal poses
        if not os.path.exists(poses_path):
            benchmark.get_logger().error(f"Poses file not found: {args.poses}")
            benchmark.get_logger().info("Try generating poses first with:")
            benchmark.get_logger().info("  ros2 run arpa_helper_tools generate_random_1000_poses.py")
            return

        goal_poses = benchmark.load_poses_from_csv(poses_path)

        for test_case in test_cases:
            # Configure motion_control_node params for this test case
            if not configure_params_for_test_case(test_case):
                benchmark.get_logger().error(f"Failed to configure params for {test_case}, skipping")
                continue

            # Run benchmark
            run_summary, detail_records = benchmark.run_benchmark(
                test_case=test_case,
                goal_poses=goal_poses,
                output_dir=args.output_dir
            )

            # Write output files
            benchmark.write_csv_files(args.output_dir, test_case, run_summary, detail_records)

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
