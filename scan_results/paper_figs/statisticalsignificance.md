# Statistical Significance — 8-Case Cost Ablation

## Kruskal-Wallis H-test (H₇, all 8 configs)

| Metric | H(7) | p |
|---|---|---|
| Manipulability Score | 2523.74 | < 0.001 |
| Plan Time (s) | 1055.65 | < 0.001 |
| Path Length (m) | 448.70 | < 0.001 |

---

## Pairwise Mann-Whitney U — with-proximity vs without-proximity (Bonferroni ×16)

**With-proximity:** proximity_only, no_area, no_joint, all_equal
**Without-proximity:** baseline, area_only, joint_only, no_proximity

| With-Prox | Without-Prox | U | p_adj (Bonf) | Sig |
|---|---|---|---|---|
| proximity_only | baseline | 804764 | 9.61e-129 | *** |
| proximity_only | area_only | 899428 | 1.04e-223 | *** |
| proximity_only | joint_only | 779996 | 2.30e-116 | *** |
| proximity_only | no_proximity | 866897 | 2.00e-198 | *** |
| no_area | baseline | 794172 | 1.33e-119 | *** |
| no_area | area_only | 880148 | 2.37e-202 | *** |
| no_area | joint_only | 771110 | 6.51e-109 | *** |
| no_area | no_proximity | 852558 | 4.34e-183 | *** |
| no_joint | baseline | 734295 | 3.49e-78 | *** |
| no_joint | area_only | 836069 | 2.43e-161 | *** |
| no_joint | joint_only | 708530 | 1.38e-67 | *** |
| no_joint | no_proximity | 804298 | 3.61e-140 | *** |
| all_equal | baseline | 715413 | 9.18e-66 | *** |
| all_equal | area_only | 815470 | 1.61e-141 | *** |
| all_equal | joint_only | 690542 | 1.74e-56 | *** |
| all_equal | no_proximity | 786036 | 1.55e-123 | *** |

**Pooled (all with-prox vs all without-prox):** U = 12,739,726, p ≈ 0 (machine zero)

---

## Ready-to-paste LaTeX

```latex
A Kruskal--Wallis test confirmed significant differences in manipulability
across the eight configurations ($H(7)=2523.7$, $p < 0.001$).
All 16 pairwise Mann--Whitney U comparisons between proximity-inclusive
and proximity-exclusive configurations remained significant after
Bonferroni correction ($p < 0.001$ in all cases).
```
