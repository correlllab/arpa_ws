#!/usr/bin/env python3
"""Generate 2x2 boxplot figure comparing IK cost configurations for IEEE CASE paper."""

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Apply IEEE style before importing shared constants
from paper_style import apply_style
apply_style()
from paper_style import *

# ---------------------------------------------------------------------------
# Load data
# ---------------------------------------------------------------------------
frames = {}
for case in COST_CASES:
    path = os.path.join(DATA_DIR, f'benchmark_cost_{case}_detail.csv')
    df = pd.read_csv(path)
    frames[case] = df[df['success'] == 1]

# ---------------------------------------------------------------------------
# Figure setup
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(2, 2, figsize=(IEEE_TEXT_W, 4.5))

panels = [
    ('plan_time_s',           'Plan Time (s)',          '(a)', True),
    ('execution_time_s',      'Execution Time (s)',     '(b)', False),
    ('path_length_m',         'Path Length (m)',         '(c)', False),
    ('manipulability_score',  'Manipulability Score',   '(d)', False),
]

BOX_COLOR = '#A8D8EA'

for ax, (col, ylabel, label, use_log) in zip(axes.flat, panels):
    data = [frames[case][col].dropna().values for case in COST_CASES]

    bp = ax.boxplot(
        data,
        patch_artist=True,
        showfliers=True,
        flierprops={'marker': '.', 'markersize': 2, 'alpha': 0.5},
        medianprops={'color': 'black', 'linewidth': 1.2},
    )

    for box in bp['boxes']:
        box.set_facecolor(BOX_COLOR)

    if use_log:
        ax.set_yscale('log')

    ax.set_ylabel(ylabel)
    ax.set_xticks(range(1, len(COST_LABELS) + 1))
    ax.set_xticklabels(COST_LABELS, rotation=30, ha='right')

    # Subplot label in top-left
    ax.text(0.03, 0.95, label, transform=ax.transAxes,
            fontsize=9, fontweight='bold', va='top', ha='left')

plt.tight_layout()
fig.savefig(os.path.join(OUT_DIR, 'fig_cost_boxplots.png'))
print(f"Saved to {os.path.join(OUT_DIR, 'fig_cost_boxplots.png')}")
