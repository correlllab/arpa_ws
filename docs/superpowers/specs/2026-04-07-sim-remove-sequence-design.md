# Sim Remove Sequence — Design Spec

## Purpose

Simulate the full part-removal sequence in Gazebo/RViz for demo purposes (e.g., speech backdrop). The arm visits every part in the parts list, hovers above it, descends to simulate unscrewing, waits, retracts, and moves to the next part. No hardware calls — uses only MoveIt planning services that already work in sim.

## Sequence Per Part (matches bt_executor pattern)

For each of the 102 parts in `hyundai_ioniq_parts_list.json`:

1. **Plan to hover** — 3cm above part position (Z = 0.91 + 0.03 = 0.94), tool-down orientation
2. **Execute** the plan
3. **Descend** — Cartesian plan down 3cm to Z = 0.91 (use `use_cartesian = true` on PlanToPose)
4. **Execute** the plan
5. **Wait 4 seconds** — simulating unscrewing
6. **Retract** — Cartesian plan back up 3cm to Z = 0.94
7. **Execute** the plan
8. Proceed to next part

### Orientation Computation

Same as `_remove_part_cb` in `core_functionality_node.py` (lines 233-244):
- `z_hat = [0, 0, -1]` (tool pointing down)
- `y_hat = normalize([-x, -y, 0])` (toward origin)
- `x_hat = cross(y_hat, z_hat)`
- Quaternion from rotation matrix `[x_hat, y_hat, z_hat]`

### Constants

| Constant | Value | Source |
|----------|-------|--------|
| `Z_HEIGHT` | 0.91 m | `core_functionality_node.py` line 232 |
| `Z_OFFSET` | 0.03 m | `bt_executor_node.py` line 28 |
| `WAIT_SECONDS` | 4 s | `bt_executor_node.py` line 29 |
| `FRAME_ID` | `floor_link` | `core_functionality_node.py` line 37 |

## Architecture

### New Node: `sim_remove_sequence_node.py`

**Location:** `src/arpa_helper_tools/scripts/sim_remove_sequence_node.py`

**Services it provides:**
- `run_sim_remove_sequence` (`std_srvs/Trigger`) — starts the sequence, blocks until complete

**Services it calls (existing):**
- `plan_to_pose` (`arpa_control/srv/PlanToPose`) — MoveIt planning
- `execute_plan` (`arpa_control/srv/ExecutePlan`) — MoveIt execution

**Topics it publishes to:**
- `/triggered_behavior` (`std_msgs/String`) — progress updates like `"Sim Remove: Part 3/102 (Nut_2) — hovering"`

**Callback group:** `ReentrantCallbackGroup` (same pattern as `core_functionality_node.py`) so the service callback can block while plan/exec futures complete.

**Executor:** `MultiThreadedExecutor` (required for reentrant callbacks).

**Parts list loading:** Reads `hyundai_ioniq_parts_list.json` from the package share directory (same as `parts_visualizer_node.py`).

**Error handling:** If planning fails for a part, log a warning and skip to the next part (don't abort the whole sequence for one failure).

### GUI Changes

**Files:** `pose_window.hpp`, `pose_window.cpp`

**New UI elements (after humanoid group box, before `leftLayout->addStretch()`):**
- `QGroupBox` titled "Sim Remove Sequence"
- `QPushButton` "Run Sim Remove Sequence" — calls the Trigger service
- Button disables while running, shows "Running... (X/102)"

**New members in `PoseWindow`:**
- `QGroupBox *m_sim_remove_group`
- `QPushButton *m_sim_remove_btn`
- `rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_sim_remove_client`
- `bool m_sim_remove_running`

**Service client:** `run_sim_remove_sequence` (`std_srvs/Trigger`)

**Status:** Logs to both `m_status_log` and `m_bt_status_monitor`.

### Launch Integration

The new node needs to be included in the sim launch file. Add it to whichever launch file starts the sim stack (e.g., `arpa_sim.launch.py` or equivalent). It only depends on `plan_to_pose` and `execute_plan` being available.

### CMakeLists / setup.py

Add `sim_remove_sequence_node.py` as an installed script in `arpa_helper_tools`'s build config.

## What This Does NOT Do

- No motor control, ZForce, visual servo, behavior triggers
- No visual servo / second sight alignment
- No collision object management
- No TSP/routing optimization (iterates parts in JSON order)
- No new service message definitions (uses existing `std_srvs/Trigger`)
