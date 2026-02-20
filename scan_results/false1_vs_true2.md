# Battery scan comparison: false1 vs true2

## false1 (run you just saved)
- **Start pose:** (0.859, 0.442, 1.244)
- **Skipped:** 3 poses (9, 24, 61)
- **Completed first pass:** 61/64
- **Errors:**
  - "Failed to configure for planning": 2 (poses 9, 24)
  - "Planning failed": 1 (pose 61)
- **End:** Battery scan complete, Removed collision plane (then terminate/Aborted)

## true2 (earlier run)
- **Start pose:** (-0.873, 0.631, 1.253)
- **Skipped:** 6 poses (8, 9, 32, 40, 53, 63)
- **Completed first pass:** 58/64
- **Errors:**
  - "Planning failed": 2 (poses 8, 9 — both at y=-0.743)
  - "Failed to configure for planning": 4 (poses 32, 40, 53, 63)

## Side-by-side

| Metric               | false1 | true2 |
|----------------------|--------|-------|
| Total poses          | 64     | 64    |
| Skipped              | 3      | 6     |
| Completed first pass | 61     | 58    |
| "Failed to configure"| 2      | 4     |
| "Planning failed"    | 1      | 2     |

## Summary

- **false1 is clearly better:** 3 skips vs 6, 61/64 vs 58/64 completed on first pass.
- **Same scan, different start:** false1 starts near center (0.86, 0.44); true2 starts at the corner (-0.87, 0.63). Traversal order is different, so the same “hard” poses get hit at different times and from different previous poses.
- **Why false1 does better:** Starting near center (false1) means the first row at y=-0.743 is hit after already doing the right-hand column (poses 1–8 succeed, including 8 at 1.109,-0.743). So pose 9 (1.109,-0.547) is the first failure. In true2, starting at (-0.87, 0.63), the path goes the other way and hits the y=-0.743 row early (poses 8 and 9 fail there). So start position changes which segment is attempted first and how many attempts succeed.
- **Sticky poses:** false1 still fails at 9 (1.109,-0.547), 24 (0.543,-0.743), 61 (-0.873, 0.042). true2 fails at 8, 9 (y=-0.743), 32, 40, 53, 63. Overlap: pose 9 fails in both; pose 24 in false1 is in the same y=-0.743 band that hurts true2; pose 61 in false1 is “Planning failed” near the left edge.
- **Conclusion:** false1 is the better run (fewer skips, more completes) mainly because of start pose and traversal order. Same corridor/planner; the (0.86, 0.44) start is more favorable for this grid than (-0.87, 0.63).
