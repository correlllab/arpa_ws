#!/usr/bin/env python3
"""
Manipulability bar chart with significance bracket comparing
with-proximity vs without-proximity cost configurations.
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import stats

sys.path.insert(0, os.path.dirname(__file__))
from paper_style import apply_style, IEEE_COL_W, COST_CASES, COST_LABELS, DATA_DIR, OUT_DIR

apply_style()

# ── Group definitions ──────────────────────────────────────────────────────────
WITH_PROX    = {'proximity_only', 'no_area', 'no_joint', 'all_equal'}
WITHOUT_PROX = {'baseline', 'area_only', 'joint_only', 'no_proximity'}

COLOR_WITH    = '#E8833A'   # orange — proximity included
COLOR_WITHOUT = '#5B8DB8'   # steel blue — proximity excluded

# ── Load data ──────────────────────────────────────────────────────────────────
means, sems, raw = [], [], []
for case in COST_CASES:
    df = pd.read_csv(os.path.join(DATA_DIR, f'benchmark_cost_{case}_detail.csv'),
                     skipinitialspace=True)
    succ = df[df['success'] == 1]['manipulability_score'].dropna()
    means.append(succ.mean())
    sems.append(succ.std() / np.sqrt(len(succ)))   # ±1 std-error
    raw.append(succ.values)

means = np.array(means)
sems  = np.array(sems)
x     = np.arange(len(COST_CASES))

colors = [COLOR_WITH if c in WITH_PROX else COLOR_WITHOUT for c in COST_CASES]

# ── Figure ─────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(IEEE_COL_W, 2.8))

bars = ax.bar(x, means, yerr=sems, capsize=3,
              color=colors, edgecolor='white', linewidth=0.5,
              error_kw={'linewidth': 0.8, 'ecolor': '#555555'})

ax.set_xticks(x)
ax.set_xticklabels(COST_LABELS, rotation=30, ha='right')
ax.set_ylabel('Manipulability Score')
ax.yaxis.grid(True, linewidth=0.3, alpha=0.7)
ax.set_axisbelow(True)

# ── Legend ─────────────────────────────────────────────────────────────────────
from matplotlib.patches import Patch
legend_elements = [
    Patch(facecolor=COLOR_WITH,    edgecolor='white', label='With proximity'),
    Patch(facecolor=COLOR_WITHOUT, edgecolor='white', label='Without proximity'),
]
ax.legend(handles=legend_elements, fontsize=6, loc='upper left',
          framealpha=0.7, edgecolor='none')

# ── Significance bracket ───────────────────────────────────────────────────────
# Compute pooled Mann-Whitney U to confirm direction
with_vals    = np.concatenate([raw[i] for i, c in enumerate(COST_CASES) if c in WITH_PROX])
without_vals = np.concatenate([raw[i] for i, c in enumerate(COST_CASES) if c in WITHOUT_PROX])
_, p_pool = stats.mannwhitneyu(with_vals, without_vals, alternative='greater')
sig_label = '***' if p_pool < 0.001 else ('**' if p_pool < 0.01 else '*')

# Bracket geometry: connect midpoint of without-prox group to midpoint of with-prox group
without_xs = [i for i, c in enumerate(COST_CASES) if c in WITHOUT_PROX]
with_xs    = [i for i, c in enumerate(COST_CASES) if c in WITH_PROX]

x_left  = np.mean(without_xs)   # midpoint of without-prox bars
x_right = np.mean(with_xs)      # midpoint of with-prox bars
x_mid   = (x_left + x_right) / 2.0

# Place bracket above tallest bar + error bar
y_top   = (means + sems).max()
y_tick  = y_top * 1.04    # where vertical ticks end
y_line  = y_top * 1.07    # horizontal connecting line
y_label = y_top * 1.10    # text label

tick_len = y_top * 0.03

# Left vertical tick
ax.plot([x_left, x_left], [y_tick, y_line], color='black', linewidth=0.8)
# Right vertical tick
ax.plot([x_right, x_right], [y_tick, y_line], color='black', linewidth=0.8)
# Horizontal line
ax.plot([x_left, x_right], [y_line, y_line], color='black', linewidth=0.8)
# Significance label
ax.text(x_mid, y_label, sig_label, ha='center', va='bottom',
        fontsize=9, fontweight='bold', color='black')

ax.set_ylim(0, y_top * 1.22)

fig.tight_layout()
os.makedirs(OUT_DIR, exist_ok=True)
out_path = os.path.join(OUT_DIR, 'fig_manip_bar_sig.png')
fig.savefig(out_path)
print(f'Saved {out_path}')
