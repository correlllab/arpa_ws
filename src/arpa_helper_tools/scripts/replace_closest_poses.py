#!/usr/bin/env python3
import math

def load_poses(path):
    poses = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            vals = [float(v) for v in line.split(',')]
            poses.append(vals)
    return poses

def xyz_dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

world_path    = '/home/thorman/arpa_ws/src/arpa_helper_tools/scripts/world_to_test_ratchet_extension_link.txt'
inprog_path   = '/home/thorman/arpa_ws/src/arpa_helper_tools/scripts/inprogress_upworld_to_test_ratchet_extension_link.txt'

world_poses  = load_poses(world_path)
inprog_poses = load_poses(inprog_path)

print(f"Loaded {len(world_poses)} world poses, {len(inprog_poses)} inprogress poses")

replacements = {}  # inprog_idx -> world_pose (last closest wins if multiple map to same)

for w_pose in world_poses:
    best_idx  = min(range(len(inprog_poses)), key=lambda i: xyz_dist(w_pose, inprog_poses[i]))
    best_dist = xyz_dist(w_pose, inprog_poses[best_idx])
    print(f"  world ({w_pose[0]:.4f},{w_pose[1]:.4f},{w_pose[2]:.4f}) -> inprog[{best_idx}] "
          f"({inprog_poses[best_idx][0]:.4f},{inprog_poses[best_idx][1]:.4f},{inprog_poses[best_idx][2]:.4f}) "
          f"dist={best_dist:.6f}")
    replacements[best_idx] = w_pose

print(f"\nReplacing {len(replacements)} entries in inprogress file")

# Apply replacements
for idx, pose in replacements.items():
    inprog_poses[idx] = pose

# Write back
with open(inprog_path, 'w') as f:
    f.write("# Poses from world to test_ratchet_extension_link\n")
    f.write("# x,y,z,qx,qy,qz,qw\n")
    for pose in inprog_poses:
        f.write(','.join(repr(v) for v in pose) + '\n')

print("Done. inprogress file updated.")
