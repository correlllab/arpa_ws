"""Spatial failure comparison: RRT vs Corridor (2-panel)."""
import os
import pandas as pd
import matplotlib.pyplot as plt
from paper_style import IEEE_TEXT_W, OUT_DIR, RRT_DETAIL, COR_DETAIL, apply_style

apply_style()

rrt = pd.read_csv(RRT_DETAIL, skipinitialspace=True)
cor = pd.read_csv(COR_DETAIL, skipinitialspace=True)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(IEEE_TEXT_W, 3.0))

for ax, df, title in [(ax1, rrt, 'Pure RRT'), (ax2, cor, 'Corridor')]:
    succ = df[df['success'] == 1]
    fail = df[df['success'] == 0]

    ax.scatter(succ['goal_x'], succ['goal_y'], s=4, alpha=0.2,
               color='#AAAAAA', label=f'Success ({len(succ)})', zorder=2)
    ax.scatter(fail['goal_x'], fail['goal_y'], s=20, alpha=0.8,
               color='#D32F2F', marker='x', linewidths=1.0,
               label=f'Failure ({len(fail)})', zorder=3)

    ax.set_xlabel('Goal X (m)')
    ax.set_ylabel('Goal Y (m)')
    ax.set_title(title)
    ax.legend(fontsize=6, loc='upper right')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

plt.tight_layout()
out = os.path.join(OUT_DIR, 'fig_spatial_failures.png')
fig.savefig(out)
print(f'Saved {out}')
plt.close()
