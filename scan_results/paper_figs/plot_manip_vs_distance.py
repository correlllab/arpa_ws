"""Manipulability vs cartesian distance scatter: baseline vs proximity_only."""
import os
import pandas as pd
import matplotlib.pyplot as plt
from paper_style import IEEE_COL_W, DATA_DIR, OUT_DIR, apply_style

apply_style()

base = pd.read_csv(os.path.join(DATA_DIR, 'benchmark_cost_baseline_detail.csv'),
                   skipinitialspace=True)
prox = pd.read_csv(os.path.join(DATA_DIR, 'benchmark_cost_proximity_only_detail.csv'),
                   skipinitialspace=True)

base_s = base[base['success'] == 1]
prox_s = prox[prox['success'] == 1]

fig, ax = plt.subplots(figsize=(IEEE_COL_W, 2.5))

ax.scatter(base_s['cartesian_distance_m'], base_s['manipulability_score'],
           s=6, alpha=0.3, color='#888888', label='Baseline', zorder=2)
ax.scatter(prox_s['cartesian_distance_m'], prox_s['manipulability_score'],
           s=6, alpha=0.3, color='#E8833A', label='Proximity Only', zorder=3)

ax.set_xlabel('Cartesian Distance (m)')
ax.set_ylabel('Manipulability Score')
ax.legend(markerscale=3)
ax.grid(True, alpha=0.3)

plt.tight_layout()
out = os.path.join(OUT_DIR, 'fig_manip_vs_dist.png')
fig.savefig(out)
print(f'Saved {out}')
plt.close()
