#!/usr/bin/env python3
"""
Generate box plots and summary plots from benchmark CSV files.
Supports naming: benchmark_corridor_{true|false}_special_{true|false}.csv (and _detail).
Uses only Python standard library (csv, math); outputs SVG.

Usage:
  python3 plot_benchmark.py [--input-dir scan_results] [--output-dir scan_results]
"""

import argparse
import csv
import glob
import os
import re
import sys
from collections import defaultdict


def discover_run_level_files(input_dir):
    """Find benchmark run-level CSVs: benchmark_corridor_*_special_*.csv (no _detail)."""
    pattern = os.path.join(input_dir, "benchmark_corridor_*_special_*.csv")
    candidates = glob.glob(pattern)
    return [p for p in candidates if "_detail" not in os.path.basename(p)]


def discover_detail_files(input_dir):
    """Find benchmark detail CSVs: benchmark_corridor_*_special_*_detail.csv."""
    pattern = os.path.join(input_dir, "benchmark_corridor_*_special_*_detail.csv")
    return glob.glob(pattern)


def config_label(corridor, special_logic):
    """Short label for (corridor, special_logic) for plots."""
    return f"c={corridor},s={special_logic}"


def load_run_level(input_dir):
    """Load run-level CSVs into dict keyed by (corridor, special_logic) -> list of run dicts."""
    data = defaultdict(list)
    for path in discover_run_level_files(input_dir):
        with open(path, newline="") as f:
            r = csv.DictReader(f)
            for row in r:
                corridor = row.get("corridor", "")
                special = row.get("special_logic", "")
                key = (corridor, special)
                data[key].append({
                    "completed": int(row["completed"]),
                    "skipped": int(row["skipped"]),
                    "total": int(row.get("total", 64)),
                    "wall_s": float(row["wall_s"]),
                })
    return dict(data)


def load_detail(input_dir):
    """Load detail CSVs into list of dicts with corridor and special_logic."""
    rows = []
    for path in discover_detail_files(input_dir):
        with open(path, newline="") as f:
            r = csv.DictReader(f)
            for row in r:
                rows.append({
                    "corridor": row["corridor"],
                    "special_logic": row.get("special_logic", ""),
                    "pose_index": int(row["pose_index"]),
                    "plan_time_s": float(row["plan_time_s"]),
                    "success": int(row["success"]),
                })
    return rows


def print_summary_table(run_data, total=64):
    """Print comparison table: mean wall_s, success rate, fail rate per config."""
    if not run_data:
        return
    print("\n========== Benchmark summary (scan_results) ==========")
    print(f"{'Config':<28} {'Runs':>5} {'wall_s (mean)':>14} {'success_rate':>14} {'fail_rate':>10}")
    print("-" * 72)
    for key in sorted(run_data.keys()):
        rows = run_data[key]
        label = config_label(key[0], key[1])
        n = len(rows)
        mean_wall = sum(d["wall_s"] for d in rows) / n
        mean_success = sum(d["completed"] / d.get("total", total) for d in rows) / n
        mean_fail = sum(d["skipped"] / d.get("total", total) for d in rows) / n
        print(f"{label:<28} {n:>5} {mean_wall:>14.1f} {mean_success:>13.1%} {mean_fail:>9.1%}")
    print("========================================================\n")


def quartiles(vals):
    """Return (q1, median, q3) for list of floats."""
    if not vals:
        return 0, 0, 0
    s = sorted(vals)
    n = len(s)
    def at(p):
        i = p * (n - 1)
        lo = int(i)
        hi = min(lo + 1, n - 1)
        return s[lo] + (i - lo) * (s[hi] - s[lo]) if lo != hi else s[lo]
    return at(0.25), at(0.5), at(0.75)


def box_stats(vals):
    """Return min, q1, median, q3, max."""
    if not vals:
        return 0, 0, 0, 0, 0
    s = sorted(vals)
    q1, med, q3 = quartiles(s)
    return s[0], q1, med, q3, s[-1]


def svg_boxplot(groups, titles, ylabel, filename, output_dir, y_min=None, y_max=None):
    """Write one SVG with multiple box plots. groups: { "label": [values] }."""
    n = len(groups)
    if n == 0:
        return
    all_vals = []
    for vals in groups.values():
        all_vals.extend(vals)
    if not all_vals:
        return
    y_lo = y_min if y_min is not None else min(all_vals)
    y_hi = y_max if y_max is not None else max(all_vals)
    if y_hi <= y_lo:
        y_hi = y_lo + 1
    pad = 0.1 * (y_hi - y_lo) or 1
    y_lo = y_lo - pad
    y_hi = y_hi + pad

    w = 400 + max(0, (n - 2) * 100)
    h = 280
    margin_l, margin_r = 50, 40
    margin_t, margin_b = 40, 50
    plot_w = w - margin_l - margin_r
    plot_h = h - margin_t - margin_b

    def y_to_svg(y):
        return margin_t + plot_h - (y - y_lo) / (y_hi - y_lo) * plot_h

    box_w = plot_w / (n + 1) * 0.6
    out = []
    out.append('<?xml version="1.0" encoding="UTF-8"?>')
    out.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}">')
    out.append('<style>text { font: 12px sans-serif; } .title { font: 14px sans-serif; }</style>')
    out.append(f'<text x="{w/2}" y="20" text-anchor="middle" class="title">{titles}</text>')
    out.append(f'<text x="{margin_l - 10}" y="{h/2}" text-anchor="middle" transform="rotate(-90,{margin_l-10},{h/2})">{ylabel}</text>')

    for i, (label, vals) in enumerate(groups.items()):
        if not vals:
            continue
        mn, q1, med, q3, mx = box_stats(vals)
        cx = margin_l + (i + 0.5) * (plot_w / (n + 1)) + plot_w / (n + 1) / 2
        x1 = cx - box_w / 2
        x2 = cx + box_w / 2
        y_q1 = y_to_svg(q1)
        y_q3 = y_to_svg(q3)
        y_med = y_to_svg(med)
        y_mn = y_to_svg(mn)
        y_mx = y_to_svg(mx)
        # Whiskers
        out.append(f'<line x1="{cx}" y1="{y_med}" x2="{cx}" y2="{y_mn}" stroke="#333" stroke-width="1"/>')
        out.append(f'<line x1="{cx}" y1="{y_med}" x2="{cx}" y2="{y_mx}" stroke="#333" stroke-width="1"/>')
        out.append(f'<line x1="{x1}" y1="{y_mn}" x2="{x2}" y2="{y_mn}" stroke="#333" stroke-width="1"/>')
        out.append(f'<line x1="{x1}" y1="{y_mx}" x2="{x2}" y2="{y_mx}" stroke="#333" stroke-width="1"/>')
        # Box
        out.append(f'<rect x="{x1}" y="{y_q3}" width="{box_w}" height="{y_q1 - y_q3}" fill="#cce5ff" stroke="#333" stroke-width="1"/>')
        out.append(f'<line x1="{x1}" y1="{y_med}" x2="{x2}" y2="{y_med}" stroke="#333" stroke-width="2"/>')
        out.append(f'<text x="{cx}" y="{h - 5}" text-anchor="middle">{label}</text>')

    out.append('</svg>')
    path = os.path.join(output_dir, filename)
    with open(path, "w") as f:
        f.write("\n".join(out))
    print(f"  {path}")


def svg_line_plot(x_label, y_label, series, title, filename, output_dir):
    """series: [ ("label", [(x,y), ...]), ... ]."""
    all_x = []
    all_y = []
    for _, points in series:
        for x, y in points:
            all_x.append(x)
            all_y.append(y)
    if not all_x:
        return
    x_lo, x_hi = min(all_x), max(all_x)
    y_lo, y_hi = min(all_y), max(all_y)
    if y_hi <= y_lo:
        y_hi = y_lo + 1
    y_lo = min(y_lo - 0.05, 0)
    y_hi = max(y_hi + 0.05, 1)

    w, h = 500, 300
    margin_l, margin_r = 50, 40
    margin_t, margin_b = 40, 45
    plot_w = w - margin_l - margin_r
    plot_h = h - margin_t - margin_b

    def to_svg(x, y):
        xx = margin_l + (x - x_lo) / (x_hi - x_lo or 1) * plot_w
        yy = margin_t + plot_h - (y - y_lo) / (y_hi - y_lo) * plot_h
        return xx, yy

    out = []
    out.append('<?xml version="1.0" encoding="UTF-8"?>')
    out.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}">')
    out.append('<style>text { font: 12px sans-serif; } .title { font: 14px sans-serif; }</style>')
    out.append(f'<text x="{w/2}" y="20" text-anchor="middle" class="title">{title}</text>')
    out.append(f'<text x="{margin_l - 10}" y="{h/2}" text-anchor="middle" transform="rotate(-90,{margin_l-10},{h/2})">{y_label}</text>')
    out.append(f'<text x="{w/2}" y="{h-5}" text-anchor="middle">{x_label}</text>')

    colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    for idx, (label, points) in enumerate(series):
        if not points:
            continue
        pts = [to_svg(x, y) for x, y in sorted(points, key=lambda p: p[0])]
        d = "M " + " L ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        c = colors[idx % len(colors)]
        out.append(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="2" stroke-linejoin="round"/>')
        out.append(f'<text x="{w - margin_r - 60}" y="{margin_t + 15 + idx*14}" fill="{c}">{label}</text>')

    out.append('</svg>')
    path = os.path.join(output_dir, filename)
    with open(path, "w") as f:
        f.write("\n".join(out))
    print(f"  {path}")


def main():
    parser = argparse.ArgumentParser(description="Plot benchmark CSVs (box plots, success rate).")
    parser.add_argument("--input-dir", default="scan_results", help="Directory with benchmark_*.csv")
    parser.add_argument("--output-dir", default=None, help="Directory for output (default: input-dir)")
    args = parser.parse_args()
    input_dir = os.path.abspath(args.input_dir)
    output_dir = os.path.abspath(args.output_dir or args.input_dir)
    if not os.path.isdir(input_dir):
        print(f"ERROR: Not a directory: {input_dir}", file=sys.stderr)
        sys.exit(1)
    os.makedirs(output_dir, exist_ok=True)

    run_data = load_run_level(input_dir)
    if not run_data:
        print("ERROR: No run-level CSV found (look for benchmark_corridor_*_special_*.csv).", file=sys.stderr)
        sys.exit(1)

    # Summary table: wall_s, success rate, fail rate per config
    print_summary_table(run_data)

    print("Generating plots...")

    # Run-level box plots: one SVG per metric with one box per config
    for col, ylabel, fname in [
        ("completed", "Completed (first pass)", "benchmark_boxplots_completed.svg"),
        ("skipped", "Skipped", "benchmark_boxplots_skipped.svg"),
        ("wall_s", "Wall time (s)", "benchmark_boxplots_wall_s.svg"),
    ]:
        groups = {config_label(c, s): [d[col] for d in rows] for (c, s), rows in run_data.items()}
        svg_boxplot(groups, f"Run-level: {col}", ylabel, fname, output_dir)

    detail = load_detail(input_dir)
    if detail:
        # Plan time by config (successful only)
        by_config = defaultdict(list)
        for row in detail:
            if row["success"] == 1 and row["plan_time_s"] >= 0:
                key = (row["corridor"], row["special_logic"])
                by_config[config_label(*key)].append(row["plan_time_s"])
        if by_config:
            svg_boxplot(
                by_config, "Plan time (successful plans only)", "Plan time (s)",
                "benchmark_boxplots_plan_time_by_config.svg", output_dir
            )

        # Success rate per pose (group by config)
        by_pose_config = defaultdict(list)
        for row in detail:
            key = (row["pose_index"], row["corridor"], row["special_logic"])
            by_pose_config[key].append(row["success"])
        configs_sorted = sorted({(c, s) for (_, c, s) in by_pose_config})
        series = []
        for (corridor, special) in configs_sorted:
            pts = []
            for pose in range(1, 65):
                k = (pose, corridor, special)
                if k in by_pose_config:
                    vals = by_pose_config[k]
                    pts.append((pose, sum(vals) / len(vals)))
            if pts:
                series.append((config_label(corridor, special), pts))
        if series:
            svg_line_plot(
                "Pose index", "Success rate",
                series, "Success rate per pose (over runs)",
                "benchmark_success_rate_per_pose.svg", output_dir
            )

        # Median plan time per pose (successful only), by config
        by_pose_config_pt = defaultdict(list)
        for row in detail:
            if row["success"] == 1 and row["plan_time_s"] >= 0:
                key = (row["pose_index"], row["corridor"], row["special_logic"])
                by_pose_config_pt[key].append(row["plan_time_s"])
        series = []
        for (corridor, special) in configs_sorted:
            pts = []
            for pose in range(1, 65):
                k = (pose, corridor, special)
                if k in by_pose_config_pt:
                    vals = by_pose_config_pt[k]
                    s = sorted(vals)
                    med = s[len(s) // 2] if s else 0
                    pts.append((pose, med))
            if pts:
                series.append((config_label(corridor, special), pts))
        if series:
            svg_line_plot(
                "Pose index", "Median plan time (s)",
                series, "Median plan time per pose (successful plans)",
                "benchmark_plan_time_per_pose.svg", output_dir
            )

    print("Done.")


if __name__ == "__main__":
    main()
