# Battery scan comparison: true1 vs true2

## true1 (first run)
- **Start pose:** (0.859, 0.442, 1.245)
- **Skipped:** 7 poses (13, 14, 24, 25, 37, 38, 56)
- **Completed first pass:** 57/64
- **Errors:** 4× "Failed to configure for planning", 3× "Planning failed"

## true2 (second run — log you sent)
- **Start pose:** (-0.873, 0.631, 1.253) — different (arm ended at corner after true1)
- **Skipped:** 6 poses (8, 9, 32, 40, 53, 63)
- **Completed first pass:** 58/64
- **Errors:**
  - "Planning failed": 2 (poses 8, 9 — both at y=-0.743)
  - "Failed to configure for planning": 4 (poses 32, 40, 53, 63)
- **Note:** Scan completed then `terminate called without an active exception` / `[ros2run]: Aborted` (cleanup bug, not a planning failure).

## Comparison

| Metric              | true1 | true2 |
|---------------------|-------|-------|
| Total poses         | 64    | 64    |
| Skipped             | 7     | 6     |
| Completed first pass| 57    | 58    |
| "Planning failed"   | 3     | 2     |
| "Failed to configure" | 4   | 4     |

**Summary**
- **true2 is slightly better:** one fewer skip (6 vs 7), one more pose completed on first pass (58 vs 57).
- **Failure type mix:** Same number of "Failed to configure for planning" (4); true2 has one fewer "Planning failed" (2 vs 3).
- **Start position matters:** true1 started near center (0.86, 0.44); true2 started at (-0.87, 0.63). Different start changes which segments are traversed first, so different poses fail (e.g. true2 fails at 8, 9 on the first row at y=-0.743; true1 failed at 24, 25 on another row).
- **Sticky spots:** Both runs show "Failed to configure" at several poses (often near workspace edges or specific y/x bands). "Planning failed" is concentrated near y=-0.743 in true2 (poses 8, 9) and at 24, 25, 38 in true1.
- **Post-scan crash:** true2 (and likely true1) ends with `terminate called without an active exception` after "Battery scan complete" — worth fixing in cleanup/shutdown, but it does not affect the 64-pose scan outcome.

**Conclusion:** Results are similar; true2 is marginally better. Failures are start-dependent and mostly "configure" or RRT timeouts at edge poses. Corridor tuning helped a bit; remaining issues are likely constraint setup ("Failed to configure") and a few tight segments (e.g. y=-0.743).
