#!/usr/bin/env python3
"""Per-goal plan time scatter: RRT vs Corridor, coloured by outcome."""

import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from paper_style import apply_style, IEEE_COL_W, OUT_DIR, RRT_DETAIL, COR_DETAIL
apply_style()

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ── Load data ────────────────────────────────────────────────────────────
rrt = pd.read_csv(RRT_DETAIL, skipinitialspace=True)
cor = pd.read_csv(COR_DETAIL, skipinitialspace=True)

# Merge on move_index, keeping plan_time and success from each
rrt_sub = rrt[["move_index", "plan_time_s", "success"]].rename(
    columns={"plan_time_s": "rrt_time", "success": "rrt_success"})
cor_sub = cor[["move_index", "plan_time_s", "success"]].rename(
    columns={"plan_time_s": "cor_time", "success": "cor_success"})

df = pd.merge(rrt_sub, cor_sub, on="move_index", how="outer")

# ── Classify outcomes ────────────────────────────────────────────────────
both_ok   = (df["rrt_success"] == 1) & (df["cor_success"] == 1)
rrt_only  = (df["rrt_success"] == 1) & (df["cor_success"] != 1)
cor_only  = (df["rrt_success"] != 1) & (df["cor_success"] == 1)
both_fail = (df["rrt_success"] != 1) & (df["cor_success"] != 1)

# ── Figure ───────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(IEEE_COL_W, IEEE_COL_W))

# Set log scale BEFORE plotting so auto-limits work
ax.set_xscale("log")
ax.set_yscale("log")

# Plot each category (both-success first so it sits behind)
categories = [
    (both_ok,   "Both success",          "#888888", 0.3, 8),
    (rrt_only,  "RRT success only",      "#4A90D9", 0.8, 20),
    (cor_only,  "Corridor success only", "#E8833A", 0.8, 20),
    (both_fail, "Both failed",           "#D32F2F", 0.9, 25),
]

for mask, label, color, alpha, size in categories:
    subset = df[mask]
    if subset.empty:
        continue
    ax.scatter(subset["rrt_time"], subset["cor_time"],
               s=size, c=color, alpha=alpha, label=label,
               edgecolors="none", rasterized=True)

# Set tight axis limits based on actual data range
all_times = np.concatenate([df["rrt_time"].dropna().values,
                            df["cor_time"].dropna().values])
lo = all_times.min() * 0.5
hi = all_times.max() * 2.0
ax.set_xlim(lo, hi)
ax.set_ylim(lo, hi)

# Diagonal y = x
ax.plot([lo, hi], [lo, hi], "k--", linewidth=0.6, zorder=0)

ax.set_xlabel("RRT Plan Time (s)")
ax.set_ylabel("Corridor Plan Time (s)")
ax.legend(frameon=False, loc="lower right", fontsize=6, markerscale=1.5)

fig.tight_layout()

os.makedirs(OUT_DIR, exist_ok=True)
fig.savefig(os.path.join(OUT_DIR, "fig_plantime_scatter.png"))
print(f"Saved to {OUT_DIR}/fig_plantime_scatter.png")
