#!/usr/bin/env python3
"""
Generate 1000 random 6-DOF goal poses with X, Y, and Z all randomized.

Points are kept within the planning space (between gantry and structure),
with Z varying from a lower bound up to the plane at 85 cm from the ground.
Same orientation convention as generate_random_1000_poses (tool down, random yaw).
Output: scan_results/random1000all.csv
"""

import argparse
import csv
import os
import numpy as np
from scipy.spatial.transform import Rotation as R


def generate_random_poses_all_axes(
    count=1000,
    seed=42,
    output_file=None,
    z_min=0.35,
    z_max=0.85,
):
    """
    Generate random 6-DOF poses with X, Y, Z all randomized in the workspace.

    Args:
        count:       Number of poses to generate (default 1000)
        seed:        RNG seed for reproducibility (default 42)
        output_file: Output CSV file path (default: scan_results/random1000all.csv)
        z_min:       Minimum z height in floor_link, m (default 0.35, above structure)
        z_max:       Maximum z height = plane at 85 cm from ground (default 0.85)

    Returns:
        List of poses as dicts with keys: x, y, z, qx, qy, qz, qw
    """
    np.random.seed(seed)

    # Planning space XY: same as battery footprint (between gantry and structure)
    LOWER_LEFT = [1.109, -0.743]
    UPPER_RIGHT = [-0.873, 0.631]
    x_min = min(LOWER_LEFT[0], UPPER_RIGHT[0])
    x_max = max(LOWER_LEFT[0], UPPER_RIGHT[0])
    y_min = min(LOWER_LEFT[1], UPPER_RIGHT[1])
    y_max = max(LOWER_LEFT[1], UPPER_RIGHT[1])

    # Base orientation: tool pointing straight down (same as other generator)
    base_rot = R.from_euler('xyz', [180.0, 0.0, 90.0], degrees=True)

    poses = []
    for _ in range(count):
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        z = np.random.uniform(z_min, z_max)

        yaw = np.random.uniform(0.0, 2.0 * np.pi)
        yaw_rot = R.from_euler('z', yaw)
        final_rot = yaw_rot * base_rot
        qx, qy, qz, qw = final_rot.as_quat()

        poses.append({
            'x': float(x), 'y': float(y), 'z': float(z),
            'qx': float(qx), 'qy': float(qy),
            'qz': float(qz), 'qw': float(qw),
        })

    if output_file is None:
        # Workspace scan_results: from scripts go up to workspace (src/arpa_helper_tools/scripts -> ../../../)
        output_file = os.path.join(
            os.path.dirname(__file__), '..', '..', '..', 'scan_results',
            'random1000all.csv'
        )

    output_file = os.path.abspath(output_file)
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    with open(output_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        writer.writeheader()
        writer.writerows(poses)

    print(f"Generated {count} random poses (X, Y, Z varied) with seed {seed}")
    print(f"  X:   [{x_min:.3f}, {x_max:.3f}] m")
    print(f"  Y:   [{y_min:.3f}, {y_max:.3f}] m")
    print(f"  Z:   [{z_min:.3f}, {z_max:.3f}] m (up to plane at 85 cm)")
    print(f"  Yaw: [0, 360°] (randomised)")
    print(f"  Output: {output_file}")

    return poses


def main():
    parser = argparse.ArgumentParser(
        description='Generate 1000 random poses with X, Y, Z all varied (output: scan_results/random1000all.csv)'
    )
    parser.add_argument('--count', type=int, default=1000, help='Number of poses (default: 1000)')
    parser.add_argument('--seed', type=int, default=42, help='RNG seed (default: 42)')
    parser.add_argument('--output', type=str, default=None, help='Output CSV path (default: scan_results/random1000all.csv)')
    parser.add_argument('--z-min', type=float, default=0.35, help='Min z in m (default: 0.35)')
    parser.add_argument('--z-max', type=float, default=0.85, help='Max z in m, plane at 85 cm (default: 0.85)')

    args = parser.parse_args()
    generate_random_poses_all_axes(
        count=args.count,
        seed=args.seed,
        output_file=args.output,
        z_min=args.z_min,
        z_max=args.z_max,
    )


if __name__ == '__main__':
    main()
