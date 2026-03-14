#!/usr/bin/env python3
"""Goal difficulty heatmap for IEEE CASE paper.

Scatter plot of goal_x vs goal_y coloured by the number of cost-function
cases (out of 8) that failed to reach each goal.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm
from paper_style import apply_style, IEEE_COL_W, COST_CASES, DATA_DIR, OUT_DIR

apply_style()

# ── Load & merge all 8 detail CSVs ──────────────────────────────────────
frames = []
for case in COST_CASES:
    path = f"{DATA_DIR}/benchmark_cost_{case}_detail.csv"
    df = pd.read_csv(path)
    frames.append(df)

combined = pd.concat(frames, ignore_index=True)

# ── Count failures per pose_index ────────────────────────────────────────
failure_counts = (
    combined.groupby("pose_index")["success"]
    .apply(lambda s: (s == 0).sum())
    .reset_index(name="fail_count")
)

# Grab goal coordinates (same for every case, so take from first frame)
coords = frames[0][["pose_index", "goal_x", "goal_y"]].drop_duplicates()
failure_counts = failure_counts.merge(coords, on="pose_index")

# ── Separate zero-failure and nonzero-failure sets ───────────────────────
ok = failure_counts[failure_counts["fail_count"] == 0]
hard = failure_counts[failure_counts["fail_count"] > 0]

# ── Plot ─────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(IEEE_COL_W, 3.0))

# Zero-failure goals: unobtrusive grey dots
ax.scatter(
    ok["goal_x"], ok["goal_y"],
    s=4, c="lightgrey", alpha=0.3, edgecolors="none", rasterized=True,
)

# Difficult goals: coloured by failure count
cmap = plt.cm.YlOrRd
bounds = np.arange(0.5, 9.5, 1)  # boundaries between 1..8
norm = BoundaryNorm(bounds, cmap.N)

sc = ax.scatter(
    hard["goal_x"], hard["goal_y"],
    s=14, c=hard["fail_count"], cmap=cmap, norm=norm,
    edgecolors="none", rasterized=True,
)

cbar = fig.colorbar(sc, ax=ax, ticks=np.arange(1, 9))
cbar.set_label("Failure Count")

ax.set_xlabel("Goal X (m)")
ax.set_ylabel("Goal Y (m)")
ax.set_aspect("equal")
ax.grid(True, alpha=0.25)

fig.tight_layout()
fig.savefig(f"{OUT_DIR}/fig_goal_difficulty.png")
plt.close(fig)

print(f"Saved {OUT_DIR}/fig_goal_difficulty.png")
