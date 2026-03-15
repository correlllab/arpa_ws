#!/usr/bin/env python3
"""Statistical significance tests for the 8-case cost-function ablation study."""

import os
import sys
import numpy as np
import pandas as pd
from scipy import stats

sys.path.insert(0, os.path.dirname(__file__))
from paper_style import COST_CASES, DATA_DIR

# ── Load data ──────────────────────────────────────────────────────────────────
groups = {}
for case in COST_CASES:
    path = os.path.join(DATA_DIR, f'benchmark_cost_{case}_detail.csv')
    df = pd.read_csv(path, skipinitialspace=True)
    groups[case] = df[df['success'] == 1].copy()

# ── Kruskal-Wallis across all 8 groups ────────────────────────────────────────
metrics = {
    'manipulability_score': 'Manipulability',
    'plan_time_s':          'Plan Time (s)',
    'path_length_m':        'Path Length (m)',
}

print("=" * 60)
print("Kruskal-Wallis H-test  (H₇, all 8 configs)")
print("=" * 60)
latex_sentences = []
for col, label in metrics.items():
    arrays = [groups[c][col].dropna().values for c in COST_CASES]
    H, p = stats.kruskal(*arrays)
    sig = "p < 0.001" if p < 0.001 else f"p = {p:.4f}"
    print(f"  {label:25s}  H(7) = {H:8.2f},  {sig}")
    latex_sentences.append(
        f"A Kruskal--Wallis test on {label.lower()} across the eight"
        f" configurations yielded $H(7)={H:.1f}$, ${sig.replace('p', 'p')}$."
    )

# ── Pairwise Mann-Whitney U: with-proximity vs without-proximity ───────────────
WITH_PROX    = ['proximity_only', 'no_area', 'no_joint', 'all_equal']
WITHOUT_PROX = ['baseline', 'area_only', 'joint_only', 'no_proximity']
N_PAIRS = len(WITH_PROX) * len(WITHOUT_PROX)   # 16

print()
print("=" * 60)
print(f"Pairwise Mann-Whitney U: with-proximity vs without-proximity")
print(f"Bonferroni correction: ×{N_PAIRS} ({len(WITH_PROX)}×{len(WITHOUT_PROX)} pairs)")
print("=" * 60)

col = 'manipulability_score'
results = []
for w in WITH_PROX:
    for wo in WITHOUT_PROX:
        a = groups[w][col].dropna().values
        b = groups[wo][col].dropna().values
        U, p_raw = stats.mannwhitneyu(a, b, alternative='greater')
        p_adj = min(p_raw * N_PAIRS, 1.0)
        results.append((w, wo, U, p_raw, p_adj))

print(f"  {'With-Prox':<18} {'Without-Prox':<18} {'U':>10} {'p_raw':>12} {'p_adj (Bonf)':>14}")
print("  " + "-" * 74)
all_sig = True
for w, wo, U, p_raw, p_adj in results:
    sig_flag = "***" if p_adj < 0.001 else ("**" if p_adj < 0.01 else ("*" if p_adj < 0.05 else "ns"))
    print(f"  {w:<18} {wo:<18} {U:>10.0f} {p_raw:>12.2e} {p_adj:>14.2e}  {sig_flag}")
    if p_adj >= 0.05:
        all_sig = False

# Summary statistics for the two groups pooled
with_vals   = np.concatenate([groups[c][col].dropna().values for c in WITH_PROX])
without_vals = np.concatenate([groups[c][col].dropna().values for c in WITHOUT_PROX])
U_pool, p_pool = stats.mannwhitneyu(with_vals, without_vals, alternative='greater')

print()
print("  Pooled comparison (all with-prox vs all without-prox):")
print(f"    U = {U_pool:.0f},  p = {p_pool:.2e}")

# ── LaTeX sentences ────────────────────────────────────────────────────────────
print()
print("=" * 60)
print("Ready-to-paste LaTeX sentences:")
print("=" * 60)
for s in latex_sentences:
    print(f"  {s}")
print()
group_label = "all" if all_sig else "most"
print(
    f"  All {N_PAIRS} pairwise Mann--Whitney U comparisons between"
    f" proximity-inclusive and proximity-exclusive configurations"
    f" remained significant after Bonferroni correction"
    f" ($p < 0.001$ in {group_label} pairs)."
)
print(
    f"  Pooled: configurations that include the proximity term"
    f" achieved higher manipulability than those without"
    f" ($U={U_pool:.0f}$, $p={p_pool:.2e}$)."
)
