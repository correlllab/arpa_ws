#!/usr/bin/env python3
"""Overlaid histograms of per-goal plan_time_s for RRT vs Corridor."""

import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from paper_style import apply_style, IEEE_COL_W, SCAN_DIR, OUT_DIR
apply_style()

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ── Load data ────────────────────────────────────────────────────────────
rrt = pd.read_csv(os.path.join(SCAN_DIR, "rrtdetail.csv"))
cor = pd.read_csv(os.path.join(SCAN_DIR, "positiondetail.csv"))

# Keep only successful plans
rrt = rrt[rrt["success"] == 1]
cor = cor[cor["success"] == 1]

rrt_times = rrt["plan_time_s"].values
cor_times = cor["plan_time_s"].values

# ── Figure ───────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(IEEE_COL_W, 2.5))

# Log-spaced bins spanning both datasets
all_times = np.concatenate([rrt_times, cor_times])
bins = np.logspace(np.log10(all_times.min() * 0.9),
                   np.log10(all_times.max() * 1.1), 51)

ax.hist(rrt_times, bins=bins, alpha=0.6, color="#4A90D9", label="Pure RRT")
ax.hist(cor_times, bins=bins, alpha=0.6, color="#E8833A", label="Corridor")

ax.set_xscale("log")
ax.set_xlabel("Plan Time (s)")
ax.set_ylabel("Count")

# Median lines — draw before layout so ylim is driven by histogram data
med_rrt = np.median(rrt_times)
med_cor = np.median(cor_times)
ax.axvline(med_rrt, color="#4A90D9", linestyle="--", linewidth=1.2, zorder=5)
ax.axvline(med_cor, color="#E8833A", linestyle="--", linewidth=1.2, zorder=5)


ax.legend(
    handles=[
        plt.Rectangle((0, 0), 1, 1, fc="#4A90D9", alpha=0.6, label="Pure RRT"),
        plt.Rectangle((0, 0), 1, 1, fc="#E8833A", alpha=0.6, label="Corridor"),
        plt.Line2D([0], [0], color="#4A90D9", linestyle="--", linewidth=1.2,
                   label=f"RRT med = {med_rrt:.2f} s"),
        plt.Line2D([0], [0], color="#E8833A", linestyle="--", linewidth=1.2,
                   label=f"Corr med = {med_cor:.2f} s"),
    ],
    frameon=False, fontsize=6, loc="upper right",
)
fig.tight_layout()

os.makedirs(OUT_DIR, exist_ok=True)
fig.savefig(os.path.join(OUT_DIR, "fig_rrt_corridor_hist.png"))
print(f"Saved to {OUT_DIR}/fig_rrt_corridor_hist.png")
