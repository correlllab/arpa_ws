# MoveIt Configuration Fixes for Relative Motion Planning

This document outlines the 8 key changes made to the MoveIt configuration to make relative motion planning work correctly in the ARPA system.

## 1. Frame Mismatch Fix (CRITICAL)

**Problem:** The GUI was sending target poses in `base_link`, but MoveIt's planning frame was `floor_link`. This caused planning to fail because the poses were unreachable.

**Solution:** Added TF2 transform in `motion_control_node.cpp` to convert target poses from `base_link` to `floor_link` (the planning frame):

```cpp
// Convert target pose from base_link to floor_link (planning frame)
geometry_msgs::msg::PoseStamped pose_in_base;
pose_in_base.header.frame_id = "base_link";
pose_in_base.pose = request->target_pose;

geometry_msgs::msg::PoseStamped pose_in_floor;
pose_in_floor = m_tf_buffer->transform(pose_in_base, "floor_link");
m_move_group->setPoseTarget(pose_in_floor.pose);
```

**Impact:** This was the single biggest fix that made everything work.

---

## 2. Increased IK Solver Attempts

**Problem:** The IK solver wasn't spending enough time trying to find valid solutions.

**Solution:** Updated `/home/the2xman/arpa_ws/src/arpa_moveit_config/config/kinematics.yaml`:

```yaml
kinematics_solver_timeout: 0.1  # increased from default
kinematics_solver_attempts: 10  # increased from default
```

**Impact:** Gives the IK solver more chances to find valid joint configurations.

---

## 3. Simplified planToPoseCallback()

**Problem:** Manual IK solving in the callback was causing conflicts with MoveIt's built-in IK solver.

**Solution:** Removed manual IK solving and let MoveIt handle it internally (like RViz does):

```cpp
// Old: Manual IK solving
// geometry_msgs::msg::Pose ik_pose = solveIK(request->target_pose);

// New: Let MoveIt handle IK internally
m_move_group->setPoseTarget(pose_in_floor.pose);
m_move_group->plan(plan);
```

**Impact:** Cleaner code that delegates IK to MoveIt's proven solver.

---

## 4. Added planRelativeMotion() Helper

**Problem:** Relative motions (like `testMoveUp`) needed a way to calculate target poses by applying deltas to the current pose.

**Solution:** Implemented `planRelativeMotion()` helper function in `motion_control_node.cpp`:

```cpp
// Get current end-effector pose
geometry_msgs::msg::Pose current_pose = m_move_group->getCurrentPose().pose;

// Apply delta (e.g., 0, 0, 0.01 for 1cm up)
// Create target pose = current + delta
// Compute IK for target
// Plan trajectory
```

**Impact:** Enables smooth relative motion planning like moving up 1cm from current position.

---

## 5. Created ompl_planning.yaml

**Problem:** Missing planner configurations for the `ur_manipulator` group.

**Solution:** Created `/home/the2xman/arpa_ws/src/arpa_moveit_config/config/ompl_planning.yaml` copied from the UR reference configuration with planner settings for:

- `RRTConnectkConfigDefault` - main sampling-based planner
- Other OMPL planner configurations

**Impact:** Provides proper motion planning pipeline configuration.

---

## 6. Fixed SRDF Chain Definition

**Problem:** The SRDF chain for `ur_manipulator` was pointing to the wrong tip link.

**Solution:** Updated `/home/the2xman/arpa_ws/src/arpa_moveit_config/config/arpa_system.srdf`:

```xml
<!-- Old: Wrong tip link -->
<group name="ur_manipulator">
  <chain base_link="base" tip_link="tool_holder_link" />
</group>

<!-- New: Correct tip link -->
<group name="ur_manipulator">
  <chain base_link="base" tip_link="tool0" />
</group>
```

**Impact:** Ensures MoveIt knows which link is the end-effector.

---

## 7. Added Cartesian Planning Option

**Problem:** Some motions (like moving straight up) were taking convoluted paths through joint space.

**Solution:** Added `use_cartesian` flag to `PlanToPose.srv` request and implemented Cartesian path planning in `motion_control_node.cpp`:

```cpp
if (request->use_cartesian) {
    m_move_group->computeCartesianPath(...);  // Straight-line motion
} else {
    m_move_group->plan(...);  // OMPL sampling-based planner
}
```

**Impact:** Allows selection between straight-line (Cartesian) and optimized (sampling-based) planning.

---

## 8. Added Missing MoveIt Parameters to Launch

**Problem:** The motion control node wasn't receiving all required MoveIt configuration parameters.

**Solution:** Updated `arpa_sim.launch.py` to add to `motion_control_node`:

- `robot_description_semantic` - SRDF configuration
- `robot_description_planning` - Planning-specific URDF features
- `ompl_planning_pipeline_config` - OMPL planner settings
- `trajectory_execution` - Execution controller configuration
- `moveit_controllers_config` - Controller names and types
- `planning_scene_monitor_parameters` - Scene monitoring settings
- `warehouse_ros_config` - Database configuration

**Impact:** Ensures the motion control node has all necessary MoveIt configuration to operate correctly.

---

## Summary

These 8 fixes work together to enable proper relative motion planning:
- **Fix #1** ensures poses are in the correct reference frame
- **Fixes #2, #3** make IK solving reliable
- **Fix #4** calculates proper target poses for relative motions
- **Fixes #5, #6** configure the planning pipeline correctly
- **Fix #7** enables both straight-line and optimized planning
- **Fix #8** passes all configurations to the motion control node

The **most critical fix is #1** (frame mismatch). Without it, the system cannot plan to any pose because they're all in the wrong reference frame.

