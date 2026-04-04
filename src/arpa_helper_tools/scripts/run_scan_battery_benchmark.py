#!/usr/bin/env python3
"""
Run scan_battery.py for all 8 IK seed cost configurations and log results.

Prerequisites:
    Start the sim stack first (pure RRT, no corridor):
      ros2 launch arpa_bringup arpa_sim.launch.py
    Then run this script in another terminal.

Usage:
    ros2 run arpa_helper_tools run_scan_battery_benchmark.py
    ros2 run arpa_helper_tools run_scan_battery_benchmark.py --config no_area
    ros2 run arpa_helper_tools run_scan_battery_benchmark.py --output-dir scan_results/with_noik
"""

import argparse
import csv
import os
import subprocess
import sys
import time
from pathlib import Path

MOTION_CONTROL_NODE = '/motion_control_node'

# All 8 cost configurations from the ablation study.
# Base params (pure RRT, analytical IK) are applied to all cases.
_BASE_PARAMS = {
    'use_corridor_constraint': 'false',
    'constrain_corridor_position': 'false',
    'constrain_corridor_orientation': 'false',
    'planning_time': '20.0',
    'use_analytical_ik': 'true',
}

COST_CONFIGS = {
    'base':     {'cost_w_joint': '0.0', 'cost_w_proximity': '0.0', 'cost_w_area': '0.0'},
    'area':     {'cost_w_joint': '0.0', 'cost_w_proximity': '0.0', 'cost_w_area': '1.0'},
    'joint':    {'cost_w_joint': '1.0', 'cost_w_proximity': '0.0', 'cost_w_area': '0.0'},
    'prox':     {'cost_w_joint': '0.0', 'cost_w_proximity': '1.0', 'cost_w_area': '0.0'},
    'no_area':  {'cost_w_joint': '1.0', 'cost_w_proximity': '1.0', 'cost_w_area': '0.0'},
    'no_joint': {'cost_w_joint': '0.0', 'cost_w_proximity': '1.0', 'cost_w_area': '1.0'},
    'no_prox':  {'cost_w_joint': '1.0', 'cost_w_proximity': '0.0', 'cost_w_area': '1.0'},
    'equal':    {'cost_w_joint': '1.0', 'cost_w_proximity': '1.0', 'cost_w_area': '1.0'},
}

ALL_CONFIGS = ['area', 'joint', 'prox', 'equal', 'base', 'no_area', 'no_joint', 'no_prox']


def set_params(config_name: str) -> bool:
    """Apply base params + cost weights for the given config via ros2 param set."""
    params = {**_BASE_PARAMS, **COST_CONFIGS[config_name]}
    print(f"\n--- Configuring: {config_name} ---")
    for name, value in params.items():
        cmd = ['ros2', 'param', 'set', MOTION_CONTROL_NODE, name, value]
        print(f"  {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if result.returncode != 0:
            print(f"  ERROR setting {name}={value}: {result.stderr.strip()}")
            return False
    print(f"--- Done: {config_name} ---\n")
    return True


def parse_benchmark_line(line: str) -> dict | None:
    """Parse the BENCHMARK|... line emitted by scan_battery.py --benchmark."""
    if not line.startswith('BENCHMARK|'):
        return None
    parts = line.strip().split('|')
    if len(parts) < 6:
        return None
    wall_s = float(parts[1])
    completed = int(parts[2])
    skipped = int(parts[3])
    total = int(parts[4])
    plan_times = [float(t) for t in parts[5].split(',') if t]
    successes = [int(s) for s in parts[6].split(',') if s] if len(parts) > 6 else []
    return {
        'wall_s': wall_s,
        'completed': completed,
        'skipped': skipped,
        'total': total,
        'plan_times': plan_times,
        'successes': successes,
    }


def run_scan(config_name: str) -> dict | None:
    """Run scan_battery.py --benchmark and return parsed results."""
    cmd = [
        'ros2', 'run', 'arpa_helper_tools', 'scan_battery.py',
        '--benchmark',
    ]
    print(f"Running: {' '.join(cmd)}")
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=3600,  # 1 hour max per scan
        )
    except subprocess.TimeoutExpired:
        print(f"ERROR: scan_battery timed out for config {config_name}")
        return None
    except Exception as e:
        print(f"ERROR: scan_battery failed for config {config_name}: {e}")
        return None

    # Print subprocess output for visibility
    for line in result.stdout.splitlines():
        print(f"  [scan] {line}")
    for line in result.stderr.splitlines():
        print(f"  [scan ERR] {line}")

    # Find the BENCHMARK line
    for line in result.stdout.splitlines():
        parsed = parse_benchmark_line(line)
        if parsed is not None:
            return parsed

    print(f"WARNING: No BENCHMARK line found in output for config {config_name}")
    return None


def write_results(output_dir: str, config_name: str, parsed: dict) -> None:
    """Write summary and per-pose detail CSVs."""
    os.makedirs(output_dir, exist_ok=True)
    plan_times = parsed['plan_times']
    successes = parsed['successes']
    n = len(plan_times)

    avg_plan_time = sum(plan_times) / n if n else 0.0
    success_rate = sum(successes) / len(successes) if successes else 0.0

    # Summary CSV (one row per config, appended)
    summary_csv = os.path.join(output_dir, 'scan_battery_benchmark_summary.csv')
    summary_exists = os.path.exists(summary_csv)
    with open(summary_csv, 'a', newline='') as f:
        fieldnames = ['config', 'completed', 'skipped', 'total', 'success_rate',
                      'wall_s', 'avg_plan_time_s']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not summary_exists:
            writer.writeheader()
        writer.writerow({
            'config': config_name,
            'completed': parsed['completed'],
            'skipped': parsed['skipped'],
            'total': parsed['total'],
            'success_rate': round(success_rate, 4),
            'wall_s': round(parsed['wall_s'], 2),
            'avg_plan_time_s': round(avg_plan_time, 4),
        })
    print(f"  Appended to {summary_csv}")

    # Per-pose detail CSV
    detail_csv = os.path.join(output_dir, f'scan_battery_{config_name}_detail.csv')
    with open(detail_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['pose_index', 'plan_time_s', 'success'])
        writer.writeheader()
        for i, (pt, s) in enumerate(zip(plan_times, successes)):
            writer.writerow({'pose_index': i + 1, 'plan_time_s': round(pt, 4), 'success': s})
    print(f"  Wrote detail: {detail_csv}")


def check_motion_control_node() -> bool:
    """Return True if motion_control_node is running (required for param set / scan_battery)."""
    r = subprocess.run(
        ['ros2', 'param', 'get', MOTION_CONTROL_NODE, 'use_corridor_constraint'],
        capture_output=True,
        text=True,
        timeout=10,
    )
    if r.returncode != 0 and ('Node not found' in (r.stderr or '') or 'Unknown node' in (r.stderr or '')):
        return False
    return r.returncode == 0


def main():
    parser = argparse.ArgumentParser(description='Scan battery benchmark across all 8 cost configs (pure RRT)')
    parser.add_argument('--config', choices=ALL_CONFIGS,
                        help='Run a single config instead of all 8')
    parser.add_argument('--output-dir', default='scan_results/with_noik',
                        help='Output directory for CSV files')
    args = parser.parse_args()

    if not check_motion_control_node():
        print("ERROR: motion_control_node is not running.")
        print("Start the ARPA stack first in another terminal, e.g.:")
        print("  ros2 launch arpa_bringup arpa_sim.launch.py")
        print("Wait until the stack is fully up, then run this benchmark again.")
        sys.exit(1)

    configs = [args.config] if args.config else ALL_CONFIGS

    print(f"Output directory : {args.output_dir}")
    print(f"Configs to run   : {configs}\n")

    for config_name in configs:
        print(f"\n{'='*60}")
        print(f"Config: {config_name}")
        print(f"{'='*60}")

        if not set_params(config_name):
            print(f"Skipping {config_name} due to param error.")
            continue

        parsed = run_scan(config_name)
        if parsed is None:
            print(f"Skipping {config_name}: no results.")
            continue

        print(f"\nResults for {config_name}:")
        print(f"  Completed : {parsed['completed']} / {parsed['total']}")
        print(f"  Skipped   : {parsed['skipped']}")
        print(f"  Wall time : {parsed['wall_s']:.1f}s")
        avg_pt = sum(parsed['plan_times']) / len(parsed['plan_times']) if parsed['plan_times'] else 0
        print(f"  Avg plan  : {avg_pt:.3f}s")

        write_results(args.output_dir, config_name, parsed)

    print(f"\nAll done. Results in: {args.output_dir}")


if __name__ == '__main__':
    main()
