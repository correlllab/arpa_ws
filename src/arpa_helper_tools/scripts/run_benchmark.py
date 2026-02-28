#!/usr/bin/env python3
"""
Run battery scan benchmark N times and append results to CSV files.

Assumes sim + motion_control are already running (with desired use_corridor_constraint
and use_special_logic). Match launch args to script args.

Usage:
  # Corridor false, special logic true (default):
  ros2 launch arpa_bringup arpa_sim.launch.py use_corridor_constraint:=false
  ros2 run arpa_helper_tools run_benchmark.py --corridor false --special-logic true --runs 10

  # Corridor true, special logic false:
  ros2 launch arpa_bringup arpa_sim.launch.py use_corridor_constraint:=true use_special_logic:=false
  ros2 run arpa_helper_tools run_benchmark.py --corridor true --special-logic false --runs 10

Output (CSV, in --output-dir):
  benchmark_corridor_{true|false}_special_{true|false}.csv
  benchmark_corridor_{true|false}_special_{true|false}_detail.csv
"""

import argparse
import csv
import os
import subprocess
import sys


def parse_benchmark_line(line: str):
    """Parse BENCHMARK|wall_s|completed|skipped|total|plan_times_csv|successes_csv|[positions_csv]
    positions_csv (optional): "x1,y1;x2,y2;..." in TSP order for free pose_index -> (x,y) mapping.
    """
    if not line.strip().startswith("BENCHMARK|"):
        return None
    parts = line.strip().split("|")
    if len(parts) < 7:
        return None
    wall_s = float(parts[1])
    completed = int(parts[2])
    skipped = int(parts[3])
    total = int(parts[4])
    plan_times = [float(x) for x in parts[5].split(",") if x.strip()]
    successes = [int(x) for x in parts[6].split(",") if x.strip()]
    positions = []
    if len(parts) >= 8 and parts[7].strip():
        for pair in parts[7].strip().split(";"):
            pair = pair.strip()
            if not pair:
                continue
            xy = pair.split(",", 1)
            if len(xy) == 2:
                try:
                    positions.append((float(xy[0]), float(xy[1])))
                except ValueError:
                    pass
    return {
        "wall_s": wall_s,
        "completed": completed,
        "skipped": skipped,
        "total": total,
        "plan_times": plan_times,
        "successes": successes,
        "positions": positions,
    }


def run_one_scan():
    """Run scan_battery.py --benchmark; return (stdout+stderr, returncode)."""
    cmd = ["ros2", "run", "arpa_helper_tools", "scan_battery.py", "--benchmark"]
    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=7200,
    )
    return result.stdout + result.stderr, result.returncode


def main():
    parser = argparse.ArgumentParser(
        description="Run battery scan N times and append to benchmark CSV files."
    )
    parser.add_argument(
        "--corridor",
        required=True,
        choices=["true", "false"],
        help="Corridor setting for this batch (must match running motion_control).",
    )
    parser.add_argument(
        "--special-logic",
        dest="special_logic",
        required=True,
        choices=["true", "false"],
        help="Special logic (multi-objective IK) setting for this batch (must match running motion_control).",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=10,
        help="Number of scan runs (default 10).",
    )
    parser.add_argument(
        "--output-dir",
        default="scan_results",
        help="Directory for CSV files (default scan_results). Relative to cwd.",
    )
    args = parser.parse_args()

    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    base = f"benchmark_corridor_{args.corridor}_special_{args.special_logic}"
    run_csv_path = os.path.join(output_dir, f"{base}.csv")
    detail_csv_path = os.path.join(output_dir, f"{base}_detail.csv")

    # Create or ensure header for run-level CSV
    run_file_exists = os.path.isfile(run_csv_path)
    detail_file_exists = os.path.isfile(detail_csv_path)

    for run_id in range(1, args.runs + 1):
        print(f"Run {run_id}/{args.runs} (corridor={args.corridor}, special_logic={args.special_logic})...", flush=True)
        out, ret = run_one_scan()
        data = None
        for line in out.splitlines():
            data = parse_benchmark_line(line)
            if data is not None:
                break
        if data is None:
            print(f"ERROR: No BENCHMARK line found in output. Return code: {ret}", file=sys.stderr)
            print("Last 20 lines of output:", file=sys.stderr)
            for line in out.splitlines()[-20:]:
                print(line, file=sys.stderr)
            sys.exit(1)

        # Append run-level row
        with open(run_csv_path, "a", newline="") as f:
            w = csv.writer(f)
            if not run_file_exists:
                w.writerow(["run_id", "corridor", "special_logic", "completed", "skipped", "total", "wall_s"])
                run_file_exists = True
            w.writerow([
                run_id,
                args.corridor,
                args.special_logic,
                data["completed"],
                data["skipped"],
                data["total"],
                f"{data['wall_s']:.4f}",
            ])

        # Append detail rows (one per pose); include x,y when present for free mapping
        positions = data.get("positions") or []
        has_xy_header = False
        if detail_file_exists and os.path.isfile(detail_csv_path):
            with open(detail_csv_path, "r", newline="") as f:
                first_line = f.readline()
            has_xy_header = "x" in first_line and "y" in first_line
        use_xy = bool(positions) and (not detail_file_exists or has_xy_header)
        with open(detail_csv_path, "a", newline="") as f:
            w = csv.writer(f)
            if not detail_file_exists:
                w.writerow(["run_id", "corridor", "special_logic", "pose_index", "x", "y", "plan_time_s", "success"])
                detail_file_exists = True
                use_xy = bool(positions)
            plan_times = data["plan_times"]
            successes = data["successes"]
            for idx in range(64):
                pose_index = idx + 1
                plan_time_s = plan_times[idx] if idx < len(plan_times) else -1.0
                success = successes[idx] if idx < len(successes) else 0
                if use_xy:
                    x = f"{positions[idx][0]:.4f}" if idx < len(positions) else ""
                    y = f"{positions[idx][1]:.4f}" if idx < len(positions) else ""
                    w.writerow([run_id, args.corridor, args.special_logic, pose_index, x, y, f"{plan_time_s:.4f}", success])
                else:
                    w.writerow([run_id, args.corridor, args.special_logic, pose_index, f"{plan_time_s:.4f}", success])

        print(f"  completed={data['completed']}, skipped={data['skipped']}, wall_s={data['wall_s']:.1f}", flush=True)

    print(f"Done. Wrote {run_csv_path} and {detail_csv_path}", flush=True)


if __name__ == "__main__":
    main()
