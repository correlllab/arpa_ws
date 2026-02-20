# Battery scan: false2 assessment

## Run summary
- **Start pose:** (-0.873, 0.631, 1.253) — same corner as true2
- **Total poses:** 64
- **Skipped:** 18 poses
- **Completed first pass:** 46/64
- **End:** Battery scan complete, Removed collision plane (then terminate/Aborted)

## Skipped poses and errors
| Pose | (x, y, z) | Error |
|------|-----------|--------|
| 2 | (-0.873, 0.435, 1.253) | Failed to configure for planning |
| 3 | (-0.873, 0.238, 1.253) | Planning failed |
| 4 | (-0.873, 0.042, 1.253) | Planning failed |
| 5 | (-0.873, -0.154, 1.253) | Planning failed |
| 6 | (-0.873, -0.350, 1.253) | Planning failed |
| 28 | (-0.024, -0.154, 1.253) | Planning failed |
| 29 | (-0.024, 0.042, 1.253) | Planning failed |
| 30 | (-0.024, 0.238, 1.253) | Planning failed |
| 35 | (0.260, 0.238, 1.253) | Planning failed |
| 36 | (0.260, 0.042, 1.253) | Planning failed |
| 37 | (0.260, -0.154, 1.253) | Planning failed |
| 44 | (0.543, -0.154, 1.253) | Planning failed |
| 45 | (0.543, 0.042, 1.253) | Planning failed |
| 46 | (0.543, 0.238, 1.253) | Planning failed |
| 54 | (0.826, -0.350, 1.253) | Planning failed |
| 62 | (1.109, 0.238, 1.253) | Planning failed |

**Error counts:** 1× "Failed to configure for planning", 17× "Planning failed"

## Comparison to true2 (same start)
| Metric | false2 | true2 |
|--------|--------|-------|
| Skipped | 18 | 6 |
| Completed first pass | 46 | 58 |
| "Failed to configure" | 1 | 4 |
| "Planning failed" | 17 | 2 |

**Verdict:** false2 is much worse than true2 — ~3× more skips, 12 fewer poses completed. Same start (-0.873, 0.631); difference likely due to environment/sim state or planner non-determinism.

## Notes
- Early block: poses 2–6 (left column x=-0.873) mostly fail; only pose 7 succeeds (long execution ~53 s suggests possible no-corridor fallback).
- Failure bands at x=-0.024, 0.260, 0.543, 0.826, 1.109 indicate systematic trouble from this start/corridor combo in this run.
