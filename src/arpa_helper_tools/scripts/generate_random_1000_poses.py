#!/usr/bin/env python3
"""
Generate reproducible random 6-DOF goal poses for point-to-point benchmark.

Uses a fixed RNG seed to ensure reproducibility across multiple benchmark runs.
Poses are generated within the defined workspace bounds (x, y from battery footprint;
z between z_min and z_max; orientation: tool pointing down with random yaw).
"""

import argparse
import csv
import os
import numpy as np
from scipy.spatial.transform import Rotation as R


def generate_random_poses(count=1000, seed=42, output_file=None,
                          z_min=None, z_max=None):
    """
    Generate random 6-DOF poses within workspace bounds.

    Args:
        count:       Number of poses to generate (default 1000)
        seed:        RNG seed for reproducibility (default 42)
        output_file: Output CSV file path (default: scan_results/random_1000_goals.csv)
        z_min:       Minimum z height in floor_link frame (default 0.90 m)
        z_max:       Maximum z height in floor_link frame (default 1.70 m)

    Returns:
        List of poses as dicts with keys: x, y, z, qx, qy, qz, qw
    """

    np.random.seed(seed)

    # Workspace XY bounds from scan_battery.py (battery footprint corners)
    LOWER_LEFT  = [1.109, -0.743]
    UPPER_RIGHT = [-0.873, 0.631]

    # Goal z is for wrist_3_link (planning EE). tool_head_link hangs 0.2315m below wrist_3_link.
    # To place tool_head_link 15cm above battery (0.83m):
    #   tool_head target = 0.83 + 0.15 = 0.98m
    #   wrist_3_link target = 0.98 + 0.2315 = 1.2115m
    BATTERY_HEIGHT_M   = 0.83
    SCAN_CLEARANCE_M   = 0.25
    WRIST_TO_TOOL_M    = 0.2315  # wrist_3_link to tool_head_link z offset
    Z_FIXED = BATTERY_HEIGHT_M + SCAN_CLEARANCE_M + WRIST_TO_TOOL_M  # 1.2115 m

    x_min = min(LOWER_LEFT[0], UPPER_RIGHT[0])
    x_max = max(LOWER_LEFT[0], UPPER_RIGHT[0])
    y_min = min(LOWER_LEFT[1], UPPER_RIGHT[1])
    y_max = max(LOWER_LEFT[1], UPPER_RIGHT[1])

    # Allow override via arguments; default is the fixed scan height
    z_lo = z_min if z_min is not None else Z_FIXED
    z_hi = z_max if z_max is not None else Z_FIXED

    # Base orientation: tool pointing straight down toward battery
    # RPY (180°, 0°, 90°) → tool -Z down, Y forward
    base_rot = R.from_euler('xyz', [180.0, 0.0, 90.0], degrees=True)

    poses = []
    for _ in range(count):
        # Random XY within battery footprint; Z fixed at scan height (or range if overridden)
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        z = np.random.uniform(z_lo, z_hi)

        # Random yaw: rotate around world Z axis so wrist angle varies.
        # Tool still points down; only the rotation around the approach axis changes.
        yaw = np.random.uniform(0.0, 2.0 * np.pi)
        yaw_rot = R.from_euler('z', yaw)
        final_rot = yaw_rot * base_rot
        qx, qy, qz, qw = final_rot.as_quat()   # scipy returns [x, y, z, w]

        poses.append({
            'x': float(x), 'y': float(y), 'z': float(z),
            'qx': float(qx), 'qy': float(qy),
            'qz': float(qz), 'qw': float(qw),
        })

    if output_file is None:
        output_file = os.path.join(
            os.path.dirname(__file__), '..', '..', 'scan_results',
            'random_1000_goals.csv'
        )

    output_file = os.path.abspath(output_file)
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    with open(output_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        writer.writeheader()
        writer.writerows(poses)

    z_desc = f"{Z_FIXED:.4f} m (fixed: {BATTERY_HEIGHT_M*100:.0f}cm battery + {SCAN_CLEARANCE_M*100:.0f}cm clearance + {WRIST_TO_TOOL_M*100:.2f}cm wrist offset = tool_head at {(BATTERY_HEIGHT_M+SCAN_CLEARANCE_M)*100:.0f}cm)" \
             if z_lo == z_hi else f"[{z_lo:.3f}, {z_hi:.3f}] m (randomised)"
    print(f"Generated {count} random poses with seed {seed}")
    print(f"  X:   [{x_min:.3f}, {x_max:.3f}] m (randomised)")
    print(f"  Y:   [{y_min:.3f}, {y_max:.3f}] m (randomised)")
    print(f"  Z:   {z_desc}")
    print(f"  Yaw: [0, 360°] (randomised around approach axis)")
    print(f"  Output: {output_file}")

    return poses


def main():
    parser = argparse.ArgumentParser(
        description='Generate random 6-DOF goal poses for benchmark')
    parser.add_argument('--count',  type=int,   default=1000,  help='Number of poses (default: 1000)')
    parser.add_argument('--seed',   type=int,   default=42,    help='RNG seed (default: 42)')
    parser.add_argument('--output', type=str,   default=None,  help='Output CSV file path')
    parser.add_argument('--z-min',  type=float, default=None,  help='Min z height in m (default: fixed at 0.98m = 83cm battery + 15cm clearance)')
    parser.add_argument('--z-max',  type=float, default=None,  help='Max z height in m (default: same as z-min, i.e. fixed height)')

    args = parser.parse_args()
    generate_random_poses(
        count=args.count, seed=args.seed,
        output_file=args.output,
        z_min=args.z_min, z_max=args.z_max,
    )


if __name__ == '__main__':
    main()
