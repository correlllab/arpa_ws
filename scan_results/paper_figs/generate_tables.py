"""Generate LaTeX table snippets for IEEE CASE paper."""
import os
import pandas as pd
import numpy as np
from paper_style import COST_CASES, COST_LABELS, DATA_DIR, SCAN_DIR, OUT_DIR, RRT_DETAIL, COR_DETAIL, apply_style

apply_style()

# ── Table 1: Cost Function Ablation Summary ──────────────────────────────────

rows = []
for case in COST_CASES:
    df = pd.read_csv(os.path.join(DATA_DIR, f'benchmark_cost_{case}_detail.csv'),
                     skipinitialspace=True)
    total = len(df)
    succ = df[df['success'] == 1]
    n_succ = len(succ)
    sr = n_succ / total * 100
    avg_plan = succ['plan_time_s'].mean()
    avg_exec = succ['execution_time_s'].mean()
    avg_path = succ['path_length_m'].mean()
    avg_manip = succ['manipulability_score'].mean()
    rows.append({
        'case': case, 'sr': sr,
        'plan': avg_plan, 'exec': avg_exec,
        'path': avg_path, 'manip': avg_manip
    })

summary = pd.DataFrame(rows)

# Find best and second-best for each metric
def rank_col(col, higher_better=False):
    vals = summary[col].values
    if higher_better:
        order = np.argsort(-vals)
    else:
        order = np.argsort(vals)
    best_idx, second_idx = order[0], order[1]
    return best_idx, second_idx

ranks = {
    'sr':    rank_col('sr',    higher_better=True),
    'plan':  rank_col('plan',  higher_better=False),
    'exec':  rank_col('exec',  higher_better=False),
    'path':  rank_col('path',  higher_better=False),
    'manip': rank_col('manip', higher_better=True),
}

def fmt(val, col, idx, decimals=2):
    s = f'{val:.{decimals}f}'
    best_i, second_i = ranks[col]
    if idx == best_i:
        return r'\textbf{' + s + '}'
    elif idx == second_i:
        return r'\underline{' + s + '}'
    return s

# Group ordering: baseline, then X-only (area, joint, prox), then no-X, then all_equal
group_order = [0, 1, 2, 3, 4, 5, 6, 7]  # matches COST_CASES order
midrule_after = {0, 3}  # after baseline, after proximity_only

lines = []
lines.append(r'\begin{table}[!t]')
lines.append(r'\caption{Cost function ablation results (avg.\ over successful plans).')
lines.append(r'\textbf{Bold} = best, \underline{underline} = second best.}')
lines.append(r'\label{tab:cost-ablation}')
lines.append(r'\centering')
lines.append(r'\resizebox{\columnwidth}{!}{%')
lines.append(r'\begin{tabular}{l ccccc}')
lines.append(r'\toprule')
lines.append(r'Config. & SR (\%)$\uparrow$ & Plan (s)$\downarrow$ & Exec (s)$\downarrow$ & Path (m)$\downarrow$ & Manip.$\uparrow$ \\')
lines.append(r'\midrule')

for i in group_order:
    r = summary.iloc[i]
    label = COST_LABELS[i]
    cols = [
        fmt(r['sr'],    'sr',    i, 1),
        fmt(r['plan'],  'plan',  i, 2),
        fmt(r['exec'],  'exec',  i, 2),
        fmt(r['path'],  'path',  i, 2),
        fmt(r['manip'], 'manip', i, 3),
    ]
    line = f'{label} & ' + ' & '.join(cols) + r' \\'
    lines.append(line)
    if i in midrule_after:
        lines.append(r'\midrule')

lines.append(r'\bottomrule')
lines.append(r'\end{tabular}%')
lines.append(r'}')
lines.append(r'\end{table}')

with open(os.path.join(OUT_DIR, 'tab_cost_ablation.tex'), 'w') as f:
    f.write('\n'.join(lines) + '\n')
print('Wrote tab_cost_ablation.tex')

# ── Table 2: RRT vs Corridor Summary ─────────────────────────────────────────

strategies = [
    ('Pure RRT', RRT_DETAIL),
    ('Corridor', COR_DETAIL),
]

strat_rows = []
for name, path in strategies:
    df = pd.read_csv(path, skipinitialspace=True)
    total = len(df)
    succ = df[df['success'] == 1]
    n_succ = len(succ)
    sr = n_succ / total * 100
    avg_plan = succ['plan_time_s'].mean()
    avg_exec = succ['execution_time_s'].mean()
    avg_path = succ['path_length_m'].mean()
    avg_eff = (succ['cartesian_distance_m'] / succ['path_length_m']).mean()
    strat_rows.append({
        'name': name, 'sr': sr,
        'plan': avg_plan, 'exec': avg_exec,
        'path': avg_path, 'eff': avg_eff
    })

def bold_if_better(val, other_val, higher_better, decimals=2):
    s = f'{val:.{decimals}f}'
    if higher_better and val >= other_val:
        return r'\textbf{' + s + '}'
    elif not higher_better and val <= other_val:
        return r'\textbf{' + s + '}'
    return s

r0, r1 = strat_rows[0], strat_rows[1]

lines2 = []
lines2.append(r'\begin{table}[!t]')
lines2.append(r'\caption{RRT vs.\ corridor planning comparison (avg.\ over successful plans).}')
lines2.append(r'\label{tab:rrt-corridor}')
lines2.append(r'\centering')
lines2.append(r'\begin{tabular}{l ccccc}')
lines2.append(r'\toprule')
lines2.append(r'Strategy & SR (\%)$\uparrow$ & Plan (s)$\downarrow$ & Exec (s)$\downarrow$ & Path (m)$\downarrow$ & Eff.$\uparrow$ \\')
lines2.append(r'\midrule')

for r in [r0, r1]:
    other = r1 if r is r0 else r0
    cols = [
        bold_if_better(r['sr'],   other['sr'],   True,  1),
        bold_if_better(r['plan'], other['plan'], False, 2),
        bold_if_better(r['exec'], other['exec'], False, 2),
        bold_if_better(r['path'], other['path'], False, 2),
        bold_if_better(r['eff'],  other['eff'],  True,  3),
    ]
    line = f"{r['name']} & " + ' & '.join(cols) + r' \\'
    lines2.append(line)

lines2.append(r'\bottomrule')
lines2.append(r'\end{tabular}')
lines2.append(r'\end{table}')

with open(os.path.join(OUT_DIR, 'tab_rrt_corridor.tex'), 'w') as f:
    f.write('\n'.join(lines2) + '\n')
print('Wrote tab_rrt_corridor.tex')
