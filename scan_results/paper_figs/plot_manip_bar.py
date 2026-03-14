#!/usr/bin/env python3
"""Bar chart of average manipulability score across the 8 cost configurations."""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import sys
sys.path.insert(0, os.path.dirname(__file__))
from paper_style import apply_style, IEEE_COL_W, COST_CASES, COST_LABELS, DATA_DIR, OUT_DIR

apply_style()

# ── Load data ─────────────────────────────────────────────────────────────
means, stds = [], []
for case in COST_CASES:
    df = pd.read_csv(os.path.join(DATA_DIR, f'benchmark_cost_{case}_detail.csv'),
                     skipinitialspace=True)
    succ = df[df['success'] == 1]['manipulability_score'].dropna()
    means.append(succ.mean())
    stds.append(succ.std())

means = np.array(means)
stds  = np.array(stds)
x     = np.arange(len(COST_CASES))
best  = np.argmax(means)

# ── Colors: highlight best, grey others ──────────────────────────────────
colors = ['#A8D8EA'] * len(COST_CASES)
colors[best] = '#E8833A'

# ── Figure ────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(IEEE_COL_W, 2.6))

bars = ax.bar(x, means, yerr=stds, capsize=3,
              color=colors, edgecolor='white', linewidth=0.5,
              error_kw={'linewidth': 0.8, 'ecolor': '#555555'})

ax.set_xticks(x)
ax.set_xticklabels(COST_LABELS, rotation=30, ha='right')
ax.set_ylabel('Manipulability Score')
ax.set_ylim(0, max(means + stds) * 1.18)
ax.yaxis.grid(True, linewidth=0.3, alpha=0.7)
ax.set_axisbelow(True)

# Annotate best bar
ax.text(best, means[best] + stds[best] + 0.005,
        f'{means[best]:.3f}', ha='center', va='bottom',
        fontsize=6.5, color='#E8833A', fontweight='bold')

fig.tight_layout()
os.makedirs(OUT_DIR, exist_ok=True)
fig.savefig(os.path.join(OUT_DIR, 'fig_manip_bar.png'))
print(f'Saved {OUT_DIR}/fig_manip_bar.png')
