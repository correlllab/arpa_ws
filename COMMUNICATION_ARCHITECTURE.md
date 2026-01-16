# MoveGroupInterface ↔ move_group Node Communication

## Overview
`MoveGroupInterface` (C++ class) communicates with the `move_group` node (ROS2 node) using **Action clients**.

## Communication Flow

```
GUI (arpa_gui)
    ↓
Services: /plan_to_pose, /execute_plan
    ↓
motion_control_node
    ↓
MoveGroupInterface::plan() [C++ class]
    ↓
Action Client: /move_group/plan [action]
    ↓
move_group node [ROS2 node]
    ↓
OMPL Planner → IK Solver → Planning Scene
    ↓
Action Server Response
    ↓
MoveGroupInterface returns result
```

## How MoveGroupInterface Works

1. **Action Clients** (internal to MoveGroupInterface):
   - `/move_group/plan` - Action for planning
   - `/move_group/execute` - Action for execution
   - `/move_group/move` - Action for planning + execution

2. **Topics** (monitored by MoveGroupInterface):
   - `/joint_states` - Current robot state
   - `/planning_scene` - Current planning scene
   - `/monitored_planning_scene` - Monitored scene updates

3. **Services** (used by MoveGroupInterface):
   - `/get_planning_scene` - Get current planning scene
   - `/check_state_validity` - Validate robot states
   - `/compute_ik` - IK solving (if needed)

## Key Code Location
- `MoveGroupInterface::plan()` creates an action client to `/move_group/plan`
- The action client sends a `MoveGroupInterface::Plan` request
- `move_group` node processes the request and returns a trajectory

## To Debug Planning Issues, We Need:

1. **IK Solver Test**: Can the IK solver solve the target pose?
   - Use `/compute_ik` service directly
   - Or check MoveIt logs for IK failures

2. **Current Robot State**: What are current joint values?
   - Check `/joint_states` topic
   - Use `ros2 topic echo /joint_states`

3. **Target Pose Validation**: Is the target pose reachable?
   - Compare with workspace limits
   - Check joint limits

4. **Planning Scene**: Are there collisions?
   - Use `/get_planning_scene` service
   - Check for self-collisions or environment collisions

5. **Compare with RViz**: What's different?
   - RViz uses same `move_group` node, so configuration should match
   - Check if RViz uses different tolerances or constraints


