# ARPA Robot Test Coordinates (Corrected)

**Coordinate System (from image):**
- Blue axis (up) = -Z direction (toward blue = negative Z)
- Green axis (right) = -Y direction (toward green = negative Y)
- Red axis (forward) = +X direction (toward red = positive X)

**Starting Position (from screenshot):**
- X: 0.0327 m
- Y: 0.2901 m
- Z: 1.1386 m
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Test Case 1: Move Up 0.1m Toward Blue (Arm only)
**Type:** Small vertical movement
- Target X: 0.0327 m
- Target Y: 0.2901 m
- Target Z: 1.0386 m (ΔZ: -0.1)
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Test Case 2: Move Down 0.1m Away From Blue (Arm only)
**Type:** Small vertical movement
- Target X: 0.0327 m
- Target Y: 0.2901 m
- Target Z: 1.2386 m (ΔZ: +0.1)
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Test Case 3: Move Right 0.15m Toward Green (Arm only)
**Type:** Right in Y direction (toward green)
- Target X: 0.0327 m
- Target Y: 0.1401 m (ΔY: -0.15)
- Target Z: 1.1386 m
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Test Case 4: Move Left 0.15m Away From Green (May use actuator)
**Type:** Left in Y direction - tests arm/actuator trade-off
- Target X: 0.0327 m
- Target Y: 0.4401 m (ΔY: +0.15)
- Target Z: 1.1386 m
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Test Case 5: Move Forward 0.1m Toward Red (Arm only)
**Type:** Forward in X direction (toward red)
- Target X: 0.1327 m (ΔX: +0.1)
- Target Y: 0.2901 m
- Target Z: 1.1386 m
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Test Case 6: Move Backward 0.1m Away From Red (Arm only)
**Type:** Backward in X direction (away from red)
- Target X: -0.0673 m (ΔX: -0.1)
- Target Y: 0.2901 m
- Target Z: 1.1386 m
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Test Case 7: Reach Further Forward 0.25m (Requires Actuator)
**Type:** Requires linear actuator extension
- Target X: 0.2827 m (ΔX: +0.25)
- Target Y: 0.2901 m
- Target Z: 1.1386 m
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Test Case 8: Diagonal Forward + Up
**Type:** Combined X and Z movement (toward red and blue)
- Target X: 0.1327 m (ΔX: +0.1)
- Target Y: 0.2901 m
- Target Z: 1.0386 m (ΔZ: -0.1)
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Test Case 9: 3D Movement Forward + Right + Up (Arm + Actuator)
**Type:** 3D movement requiring coordination
- Target X: 0.0827 m (ΔX: +0.05)
- Target Y: 0.1901 m (ΔY: -0.1)
- Target Z: 1.0386 m (ΔZ: -0.1)
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Test Case 10: Rotate Yaw +30°
**Type:** Rotation around Z-axis (yaw)
- Target X: 0.0327 m
- Target Y: 0.2901 m
- Target Z: 1.1386 m
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.5307 rad (ΔYaw: +0.5236)

---

## Test Case 11: Rotate Yaw -30°
**Type:** Rotation around Z-axis (negative yaw)
- Target X: 0.0327 m
- Target Y: 0.2901 m
- Target Z: 1.1386 m
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: -0.5165 rad (ΔYaw: -0.5236)

---

## Test Case 12: Rotate Roll +15°
**Type:** Rotation around X-axis (roll)
- Target X: 0.0327 m
- Target Y: 0.2901 m
- Target Z: 1.1386 m
- Roll: 1.8315 rad (ΔRoll: +0.2618)
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Test Case 13: Rotate Pitch +20°
**Type:** Rotation around Y-axis (pitch)
- Target X: 0.0327 m
- Target Y: 0.2901 m
- Target Z: 1.1386 m
- Roll: 1.5697 rad
- Pitch: 0.3877 rad (ΔPitch: +0.3491)
- Yaw: 0.0071 rad

---

## Test Case 14: Extreme Forward Reach 0.35m (Needs Actuator)
**Type:** Tests 7-DOF coordination - likely requires significant actuator extension
- Target X: 0.3827 m (ΔX: +0.35)
- Target Y: 0.2901 m
- Target Z: 1.1386 m
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Test Case 15: High Reach Forward + Up with Rotation
**Type:** Combines forward reach and upward movement with rotation (toward red and blue)
- Target X: 0.1327 m (ΔX: +0.1)
- Target Y: 0.2901 m
- Target Z: 0.9386 m (ΔZ: -0.2)
- Roll: 1.5697 rad
- Pitch: 0.2386 rad (ΔPitch: +0.2)
- Yaw: 0.0071 rad

---

## Test Case 16: Backward and Down (Tests Actuator Retraction)
**Type:** Backward and down movement - tests if IK prefers actuator retraction (away from red and blue)
- Target X: -0.0173 m (ΔX: -0.05)
- Target Y: 0.2901 m
- Target Z: 1.2886 m (ΔZ: +0.15)
- Roll: 1.5697 rad
- Pitch: 0.0386 rad
- Yaw: 0.0071 rad

---

## Testing Strategy

1. **Start with small movements (Tests 1-6):** Verify basic arm planning works
2. **Medium movements (Tests 7-9):** Should see actuator starting to be used
3. **Rotations (Tests 10-13):** Verify orientation control
4. **Extreme cases (Tests 14-16):** Confirm 7-DOF coordination
5. **Use Cartesian mode first:** Should be faster for small movements
6. **Switch to sampling-based if Cartesian fails:** For complex 7-DOF coordination

## Coordinate System Quick Reference

| Direction | Axis | Change |
|-----------|------|--------|
| Toward Blue (Up) | Z | **Negative** ΔZ |
| Away from Blue (Down) | Z | Positive ΔZ |
| Toward Green (Right) | Y | **Negative** ΔY |
| Away from Green (Left) | Y | Positive ΔY |
| Toward Red (Forward) | X | Positive ΔX |
| Away from Red (Backward) | X | Negative ΔX |

## Expected Results

| Test # | Expected Planner | Notes |
|--------|------------------|-------|
| 1-6 | Cartesian or Sampling | Arm-only, small movements |
| 7 | Sampling + Actuator | Medium reach, actuator may extend |
| 8-9 | Sampling + Actuator | Multi-axis, coordination needed |
| 10-13 | Cartesian or Sampling | Rotation focus |
| 14-16 | Sampling + Actuator | Extreme cases, critical for 7-DOF verification |
