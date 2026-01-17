# Combined 7-DOF Motion Planning: Linear Actuator + Arm Integration

## Summary

Fixed coordinated motion planning by enabling the 7 degrees of freedom (6 from UR arm + 1 from linear actuator) to work together. The arm and actuator now move as a unified system to reach targets, with automatic collision avoidance of the gantry structure.

---

## Key Changes

### 1. SRDF Group Definition (src/arpa_moveit_config/config/arpa_system.srdf)

**Problem:** Chain definition wasn't exposing all 7 joints to OMPL planner.

**Fix:** Changed from chain to explicit joint listing:
```xml
<!-- BEFORE (chain - didn't work) -->
<group name="ur16e_on_gantry">
    <chain base_link="linear_actuator_link" tip_link="tool0"/>
</group>

<!-- AFTER (explicit joints - works!) -->
<group name="ur16e_on_gantry">
    <joint name="linear_actuator_to_linear_actuator_plate_joint"/>
    <joint name="shoulder_pan_joint"/>
    <joint name="shoulder_lift_joint"/>
    <joint name="elbow_joint"/>
    <joint name="wrist_1_joint"/>
    <joint name="wrist_2_joint"/>
    <joint name="wrist_3_joint"/>
</group>
```

**Why:** Forces OMPL to explore all 7 joints simultaneously during planning.

---

### 2. Kinematics Solver Configuration (src/arpa_moveit_config/config/kinematics.yaml)

**Problem:** 6-DOF solver settings weren't sufficient for 7-DOF IK.

**Fix:**
```yaml
ur16e_on_gantry:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.5          # Increased from 0.2
  kinematics_solver_attempts: 30          # Increased from 20
  tip_frame: tool0
```

**Why:** 7-DOF IK is more complex, needs more time and attempts to find valid solutions avoiding gantry obstacles.

---

### 3. OMPL Planning Configuration (src/ur_manipulation/src/motion_control_node.cpp)

**Problem:** OMPL had insufficient time to explore 7-DOF space.

**Fix:** Increased planning time and attempts (lines ~302-303):
```cpp
m_move_group->setPlanningTime(30.0);        // Increased from 15.0
m_move_group->setNumPlanningAttempts(20);   // Increased from 10
```

**Why:** More time allows OMPL to find collision-free paths in the expanded 7-DOF joint space.

---

### 4. Workspace Bounds (src/ur_manipulation/src/motion_control_node.cpp)

**Problem:** Planner was attempting invalid regions inside gantry structure.

**Fix:** Added workspace bounds (lines ~309-311):
```cpp
m_move_group->setWorkspace(
  -1.3, -1.5, 0.2,    // min_x, min_y, min_z
   1.3,  1.5, 1.7);   // max_x, max_y, max_z
```

**Bounds Explanation** (from gantry URDF geometry):
- **X: -1.3 to +1.3** - Avoids pillars at ±1.4744m (0.17m clearance)
- **Y: -1.5 to +1.5** - Full range (no Y-axis obstacles)
- **Z: 0.2 to 1.7** - Above floor, below crossbeam at 1.9419m (0.24m clearance)

**Why:** Prevents planner from exploring impossible regions, focuses sampling on valid workspace.

---

### 5. Controller Type Fix (src/arpa_moveit_config/config/ros2_controllers.yaml)

**Problem:** MoveIt expected `FollowJointTrajectory` action, but controller was `JointGroupPositionController`.

**Fix:** Changed controller type:
```yaml
# BEFORE (couldn't execute MoveIt trajectories)
linear_actuator_controller:
  type: position_controllers/JointGroupPositionController

# AFTER (MoveIt can send trajectories)
linear_actuator_controller:
  type: joint_trajectory_controller/JointTrajectoryController
```

Added trajectory controller parameters:
```yaml
linear_actuator_controller:
  ros__parameters:
    joints:
      - linear_actuator_to_linear_actuator_plate_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: true
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.0
```

**Why:** Enables MoveIt to send coordinated 7-DOF trajectories to both arm and actuator simultaneously.

---

### 6. Motion Control Node Group (src/ur_manipulation/src/motion_control_node.cpp)

**Problem:** Only controlling arm (6-DOF), not actuator.

**Fix:** Changed planning group (line ~89):
```cpp
// Changed from:
m_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
  shared_from_this(), "ur_manipulator");

// To:
m_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
  shared_from_this(), "ur16e_on_gantry");
```

**Why:** Uses the 7-DOF group that includes both arm and linear actuator.

---

## Collision Avoidance System

### Already Working:
- Gantry collision geometry in planning scene (crossbeam 3.0988m × 0.15m × 0.15m, pillars at ±1.4744m)
- Arm self-collision checking enabled
- Gantry-to-arm collision detection enabled

### Fixes Made:
1. **Workspace bounds** prevent exploration of invalid regions
2. **IK solver timeout increase** allows finding solutions that navigate around obstacles
3. **OMPL planning time increase** explores more of the 7-DOF space
4. **Explicit joint listing** ensures all DOF are considered during planning

### How It Works:
1. User requests arm reach to target position
2. MoveIt checks if arm alone (6-DOF) can reach it without collision
3. If not, OMPL explores 7-DOF space (including linear actuator extension)
4. If valid solution found, actuator extends and arm adjusts to reach target
5. Coordinated trajectory sent to both controllers
6. Arm and actuator move together to reach target safely

---

## Testing

After all changes, the system exhibits:

✅ **Planning works:** Generates collision-free paths in 7-DOF space  
✅ **Execution works:** Sends coordinated trajectories to arm + actuator simultaneously  
✅ **Gantry avoidance:** Prevents collisions with fixed structure  
✅ **Auto actuator deployment:** Extends actuator when needed to reach targets  

### Test Case:
1. Launch: `ros2 launch arpa_bringup arpa_sim.launch.py`
2. Request arm reach to extreme forward position (Test Case 7 or 14)
3. Observe: Actuator extends automatically, arm adjusts, reaches target without collision

---

## Files Modified

| File | Change |
|------|--------|
| `arpa_system.srdf` | Chain → explicit joints for `ur16e_on_gantry` group |
| `kinematics.yaml` | IK timeout 0.2s → 0.5s, attempts 20 → 30 |
| `motion_control_node.cpp` | Planning time 15s → 30s, attempts 10 → 20, added workspace bounds |
| `ros2_controllers.yaml` | Controller type `JointGroupPositionController` → `JointTrajectoryController` |

---

## Technical Architecture

```
User Request (Target Pose)
    ↓
Motion Control Node (ur16e_on_gantry group)
    ↓
MoveIt Planning:
  - KDL IK Solver (7-DOF: finds valid joint configurations)
  - OMPL Planner (explores 7-DOF space within workspace bounds)
  - Collision Checking (gantry + self-collision)
    ↓
MoveIt Trajectory Generation
    ↓
ROS2 Controller Manager
  ├→ Joint Trajectory Controller (arm: 6-DOF)
  └→ Linear Actuator Controller (actuator: 1-DOF)
    ↓
Execution (Gazebo Simulation)
    ↓
Arm + Actuator move together to reach target
```

---

## Known Limitations

- GUI slider no longer moves actuator (now MoveIt-controlled only)
- Slider can be re-enabled by switching back to `JointGroupPositionController`, but would break MoveIt coordination
- Future enhancement: dual-mode control (slider for manual, MoveIt for planned motion)

