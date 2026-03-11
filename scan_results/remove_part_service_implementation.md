# /remove_part Service — Full Implementation Reference

## Overview
A ROS2 service in `core_functionality_node.py` that performs a simulated screw removal sequence.
When called with a part name and pose, the robot: approaches 3cm above -> descends (Cartesian) ->
motor on -> waits 4s -> motor off -> retracts (Cartesian).

---

## 1. Custom Service Definition

### File: `src/custom_ros_messages/srv/RemovePart.srv` (NEW)
```
string part_name
geometry_msgs/Pose pose
---
bool success
string message
```

### File: `src/custom_ros_messages/CMakeLists.txt` (ADD LINE)
Add `"srv/RemovePart.srv"` to the `rosidl_generate_interfaces` block, e.g. after `"srv/DetectParts.srv"`:
```cmake
  "srv/DetectParts.srv"
  "srv/RemovePart.srv"
```

### Build:
```bash
sudo colcon build --symlink-install --packages-select custom_ros_messages
```

---

## 2. Changes to core_functionality_node.py

### 2a. Import (line 8)
Change:
```python
from custom_ros_messages.srv import EthernetMotor, UR16BehaviorTrigger
```
To:
```python
from custom_ros_messages.srv import EthernetMotor, UR16BehaviorTrigger, RemovePart
```

### 2b. Constants (after BASE_FRAME/EE_FRAME definitions)
Add:
```python
Z_OFFSET_M = 0.03
REMOVE_WAIT_SECONDS = 4
MOTOR_SPEED = 100
```

### 2c. Service server in __init__ (after planning_scene_pub creation)
Add:
```python
        self.remove_part_service = self.create_service(
            RemovePart, 'remove_part', self._handle_remove_part
        )
```

### 2d. Add use_cartesian parameter to plan_to_pose()
Change signature from:
```python
    def plan_to_pose(self, x, y, z, qx, qy, qz, qw, frame_id="world"):
```
To:
```python
    def plan_to_pose(self, x, y, z, qx, qy, qz, qw, frame_id="world", use_cartesian=False):
```

Add after setting orientation fields:
```python
        req.use_cartesian = use_cartesian
```

Update the log line to include cartesian status:
```python
        self.get_logger().info(f"Planning to pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                               f"qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f} "
                               f"in frame '{frame_id}' (cartesian={use_cartesian})")
```

### 2e. Service handler method (add to CoreNode class, after get_tsp_order)
```python
    def _handle_remove_part(self, request, response):
        part_name = request.part_name
        p = request.pose.position
        o = request.pose.orientation
        x, y, z = p.x, p.y, p.z
        qx, qy, qz, qw = o.x, o.y, o.z, o.w

        self.get_logger().info(
            f"remove_part requested: '{part_name}' at "
            f"({x:.3f}, {y:.3f}, {z:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
        )

        try:
            # Step 1: Approach — plan to 3cm above part
            self.get_logger().info(f"[{part_name}] Step 1: Approach 3cm above")
            if not self.plan_to_pose(x, y, z + Z_OFFSET_M, qx, qy, qz, qw, frame_id=BASE_FRAME):
                raise RuntimeError("Plan to approach position failed")
            if not self.execute_plan():
                raise RuntimeError("Execute approach failed")

            # Step 2: Descend — Cartesian move down to part
            self.get_logger().info(f"[{part_name}] Step 2: Descend (Cartesian)")
            if not self.plan_to_pose(x, y, z, qx, qy, qz, qw, frame_id=BASE_FRAME, use_cartesian=True):
                raise RuntimeError("Plan Cartesian descend failed")
            if not self.execute_plan():
                raise RuntimeError("Execute descend failed")

            # Step 3: Motor ON
            self.get_logger().info(f"[{part_name}] Step 3: Motor ON")
            self.motor_control(MOTOR_SPEED)

            # Step 4: Wait (simulate removal)
            self.get_logger().info(f"[{part_name}] Step 4: Wait {REMOVE_WAIT_SECONDS}s")
            time.sleep(REMOVE_WAIT_SECONDS)

            # Step 5: Motor OFF
            self.get_logger().info(f"[{part_name}] Step 5: Motor OFF")
            self.motor_control(0)

            # Step 6: Retract — Cartesian move back up
            self.get_logger().info(f"[{part_name}] Step 6: Retract (Cartesian)")
            if not self.plan_to_pose(x, y, z + Z_OFFSET_M, qx, qy, qz, qw, frame_id=BASE_FRAME, use_cartesian=True):
                raise RuntimeError("Plan Cartesian retract failed")
            if not self.execute_plan():
                raise RuntimeError("Execute retract failed")

            response.success = True
            response.message = f"Part '{part_name}' removal complete"
            self.get_logger().info(f"[{part_name}] Removal sequence complete")

        except RuntimeError as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"[{part_name}] Removal failed: {e}")

        return response
```

---

## 3. Test Command
```bash
ros2 service call /remove_part custom_ros_messages/srv/RemovePart \
  "{part_name: 'test_screw', pose: {position: {x: 1.0, y: -0.5, z: 1.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## 4. Key Dependencies
- `plan_to_pose` service (from arpa_control motion_control_node) — must support `use_cartesian` field
- `execute_plan` service (from arpa_control motion_control_node)
- `motor_control` service (optional, gracefully skipped if unavailable)
- BASE_FRAME = "floor_link"
- The node must be spinning (background thread in CoreNode.__init__ handles this)
