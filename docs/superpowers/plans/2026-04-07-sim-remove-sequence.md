# Sim Remove Sequence Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a simulation-only part removal sequence that iterates all 102 parts in the parts list, hovering/descending/waiting/retracting at each one, triggered by a GUI button.

**Architecture:** New Python node `sim_remove_sequence_node.py` exposes a `std_srvs/Trigger` service. It loads `hyundai_ioniq_parts_list.json`, computes tool-down orientations per part (same as `_remove_part_cb`), and calls existing `plan_to_pose` + `execute_plan` MoveIt services. GUI gets a new button after the humanoid section.

**Tech Stack:** ROS2 Humble, Python 3 (node), C++ / Qt5 (GUI), ament_cmake

---

## File Map

| Action | File | Responsibility |
|--------|------|----------------|
| Create | `src/arpa_helper_tools/scripts/sim_remove_sequence_node.py` | Python node: load parts, expose Trigger service, run sequence |
| Modify | `src/arpa_helper_tools/CMakeLists.txt:14-26` | Register new script for install |
| Modify | `src/arpa_gui/include/arpa_gui/pose_window.hpp` | Declare new button, client, group box, running flag |
| Modify | `src/arpa_gui/src/pose_window.cpp` | Add UI elements, connect button, implement callback |
| Modify | `src/arpa_bringup/launch/arpa_sim.launch.py:384-414` | Add node to sim launch |

---

### Task 1: Create `sim_remove_sequence_node.py`

**Files:**
- Create: `src/arpa_helper_tools/scripts/sim_remove_sequence_node.py`

- [ ] **Step 1: Create the node file**

```python
#!/usr/bin/env python3
"""
Sim Remove Sequence node: iterates all parts in the parts list,
planning hover -> descend -> wait -> retract for each part.
Uses only MoveIt planning services (no hardware calls).
"""

import os
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from arpa_control.srv import PlanToPose, ExecutePlan
from moveit_msgs.msg import Constraints
from ament_index_python.packages import get_package_share_directory

Z_HEIGHT = 0.91
Z_OFFSET_M = 0.03
WAIT_SECONDS = 4
FRAME_ID = "floor_link"


def compute_tool_down_quaternion(x, y):
    """
    Compute quaternion for tool-pointing-down orientation,
    same logic as _remove_part_cb in core_functionality_node.py.
    z_hat = [0, 0, -1], y_hat = normalize([-x, -y, 0]), x_hat = cross(y, z).
    Returns (qx, qy, qz, qw).
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    z_hat = np.array([0.0, 0.0, -1.0])
    toward_origin = np.array([-x, -y, 0.0])
    norm = np.linalg.norm(toward_origin)
    y_hat = toward_origin / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0])
    x_hat = np.cross(y_hat, z_hat)
    R = np.column_stack([x_hat, y_hat, z_hat])
    qx, qy, qz, qw = Rotation.from_matrix(R).as_quat()
    return float(qx), float(qy), float(qz), float(qw)


def make_pose_stamped(frame_id, x, y, z, qx, qy, qz, qw):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp.sec = 0
    ps.header.stamp.nanosec = 0
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


class SimRemoveSequenceNode(Node):
    def __init__(self):
        super().__init__('sim_remove_sequence_node')

        self._cb_group = ReentrantCallbackGroup()

        self._plan_client = self.create_client(
            PlanToPose, 'plan_to_pose', callback_group=self._cb_group
        )
        self._exec_client = self.create_client(
            ExecutePlan, 'execute_plan', callback_group=self._cb_group
        )
        self._progress_pub = self.create_publisher(String, '/triggered_behavior', 10)

        self.create_service(
            Trigger, 'run_sim_remove_sequence',
            self._handle_run, callback_group=self._cb_group
        )

        self._parts = self._load_parts()
        self.get_logger().info(
            f'sim_remove_sequence_node ready: {len(self._parts)} parts loaded. '
            'Call run_sim_remove_sequence to start.'
        )

    def _load_parts(self):
        try:
            pkg_share = get_package_share_directory("arpa_helper_tools")
        except Exception:
            pkg_share = os.path.join(os.path.dirname(__file__), "..", "resources")
        json_path = os.path.join(
            pkg_share, "resources", "parts_lists", "hyundai_ioniq_parts_list.json"
        )
        if not os.path.isfile(json_path):
            self.get_logger().error(f"Parts list not found at {json_path}")
            return {}
        with open(json_path) as f:
            return json.load(f)

    def _publish_progress(self, msg):
        self._progress_pub.publish(String(data=msg))
        self.get_logger().info(msg)

    def _plan_and_execute(self, pose_stamped, use_cartesian=False):
        req = PlanToPose.Request()
        req.target_pose = pose_stamped
        req.use_cartesian = bool(use_cartesian)
        req.path_constraints = Constraints()

        if not self._plan_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('plan_to_pose service not available')
        future = self._plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done():
            raise RuntimeError('plan_to_pose timed out')
        result = future.result()
        if not result.success:
            raise RuntimeError(f'plan_to_pose failed: {result.message}')

        if not self._exec_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('execute_plan service not available')
        exec_future = self._exec_client.call_async(ExecutePlan.Request())
        rclpy.spin_until_future_complete(self, exec_future, timeout_sec=60.0)
        if not exec_future.done():
            raise RuntimeError('execute_plan timed out')
        exec_result = exec_future.result()
        if not exec_result.success:
            raise RuntimeError(f'execute_plan failed: {exec_result.message}')

    def _handle_run(self, request, response):
        del request
        if not self._parts:
            response.success = False
            response.message = 'No parts loaded'
            return response

        if not self._plan_client.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = 'plan_to_pose service not available'
            return response

        n = len(self._parts)
        failed = []

        for i, (name, coords) in enumerate(self._parts.items()):
            part_num = i + 1
            x, y, z_raw = coords
            qx, qy, qz, qw = compute_tool_down_quaternion(x, y)

            hover_z = Z_HEIGHT + Z_OFFSET_M
            at_z = Z_HEIGHT

            hover_pose = make_pose_stamped(FRAME_ID, x, y, hover_z, qx, qy, qz, qw)
            at_pose = make_pose_stamped(FRAME_ID, x, y, at_z, qx, qy, qz, qw)

            try:
                # Step 1: Plan to hover position (free-space)
                self._publish_progress(
                    f'Sim Remove: Part {part_num}/{n} ({name}) — hovering'
                )
                self._plan_and_execute(hover_pose, use_cartesian=False)

                # Step 2: Descend to part (Cartesian)
                self._publish_progress(
                    f'Sim Remove: Part {part_num}/{n} ({name}) — descending'
                )
                self._plan_and_execute(at_pose, use_cartesian=True)

                # Step 3: Wait (simulate unscrewing)
                self._publish_progress(
                    f'Sim Remove: Part {part_num}/{n} ({name}) — unscrewing ({WAIT_SECONDS}s)'
                )
                time.sleep(WAIT_SECONDS)

                # Step 4: Retract (Cartesian)
                self._publish_progress(
                    f'Sim Remove: Part {part_num}/{n} ({name}) — retracting'
                )
                self._plan_and_execute(hover_pose, use_cartesian=True)

            except RuntimeError as e:
                self.get_logger().warn(
                    f'Part {part_num}/{n} ({name}) failed: {e} — skipping'
                )
                self._publish_progress(
                    f'Sim Remove: Part {part_num}/{n} ({name}) FAILED — skipping'
                )
                failed.append(name)
                continue

        if failed:
            response.success = True
            response.message = (
                f'Sequence done. {n - len(failed)}/{n} succeeded. '
                f'Failed: {", ".join(failed)}'
            )
        else:
            response.success = True
            response.message = f'Sequence complete. All {n} parts done.'

        self._publish_progress(f'Sim Remove: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimRemoveSequenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Make the script executable**

Run: `chmod +x src/arpa_helper_tools/scripts/sim_remove_sequence_node.py`

- [ ] **Step 3: Commit**

```bash
git add src/arpa_helper_tools/scripts/sim_remove_sequence_node.py
git commit -m "feat: add sim_remove_sequence_node for simulation part removal demo"
```

---

### Task 2: Register node in CMakeLists.txt

**Files:**
- Modify: `src/arpa_helper_tools/CMakeLists.txt:14-26`

- [ ] **Step 1: Add the new script to the install list**

In `src/arpa_helper_tools/CMakeLists.txt`, add `scripts/sim_remove_sequence_node.py` to the `install(PROGRAMS ...)` block, after `scripts/battery_pointcloud_publisher.py`:

```cmake
install(PROGRAMS
  scripts/record_poses.py
  scripts/remove_screws_from_recorded_poses.py
  scripts/remove_screws_from_vision.py
  scripts/core_functionality_node.py
  scripts/scan_battery_action_server.py
  scripts/scan_battery.py
  scripts/run_benchmark.py
  scripts/plot_benchmark.py
  scripts/vla_data_collection_node.py
  scripts/parts_visualizer_node.py
  scripts/battery_pointcloud_publisher.py
  scripts/sim_remove_sequence_node.py
  DESTINATION lib/${PROJECT_NAME}
)
```

- [ ] **Step 2: Commit**

```bash
git add src/arpa_helper_tools/CMakeLists.txt
git commit -m "build: register sim_remove_sequence_node in CMakeLists"
```

---

### Task 3: Add node to sim launch file

**Files:**
- Modify: `src/arpa_bringup/launch/arpa_sim.launch.py:384-414`

- [ ] **Step 1: Add the node definition and include it in the return list**

After the `battery_pc_publisher` Node definition (around line 396), add:

```python
    sim_remove_sequence_node = Node(
        package="arpa_helper_tools",
        executable="sim_remove_sequence_node.py",
        name="sim_remove_sequence_node",
        output="screen",
    )
```

Then add `sim_remove_sequence_node` to the `to_return` list (after `battery_pc_publisher`):

```python
    to_return = [
        arpa_sim_control_launch,
        delayed_moveit_and_motion,
        rosbridge_mcp,
        static_tf_world_to_floor,
        humanoid_spawn_launch,
        parts_visualizer_node,
        battery_pc_publisher,
        sim_remove_sequence_node,
        arpa_gui_launch,
    ]
```

- [ ] **Step 2: Commit**

```bash
git add src/arpa_bringup/launch/arpa_sim.launch.py
git commit -m "launch: include sim_remove_sequence_node in arpa_sim.launch"
```

---

### Task 4: Add GUI button — header declarations

**Files:**
- Modify: `src/arpa_gui/include/arpa_gui/pose_window.hpp`

- [ ] **Step 1: Add slot declaration**

After `void togglePointCloud();` (line 66), add:

```cpp
    void runSimRemoveSequence();
```

- [ ] **Step 2: Add member declarations**

After the humanoid section members (after line 179, before the closing `};`), add:

```cpp
    // ============ SIM REMOVE SEQUENCE ============
    QGroupBox *m_sim_remove_group;
    QPushButton *m_sim_remove_btn;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_sim_remove_client;
    bool m_sim_remove_running{false};
```

- [ ] **Step 3: Commit**

```bash
git add src/arpa_gui/include/arpa_gui/pose_window.hpp
git commit -m "gui: declare sim remove sequence UI members in header"
```

---

### Task 5: Add GUI button — implementation

**Files:**
- Modify: `src/arpa_gui/src/pose_window.cpp`

- [ ] **Step 1: Add service client creation in constructor**

After the humanoid publisher lines (after line 58), add:

```cpp
    // Sim remove sequence client
    m_sim_remove_client = m_node->create_client<std_srvs::srv::Trigger>("run_sim_remove_sequence");
```

- [ ] **Step 2: Add UI elements in setupUI()**

After `leftLayout->addWidget(m_humanoid_group);` (line 371) and before `leftLayout->addStretch();` (line 373), add:

```cpp
    // ============ SIM REMOVE SEQUENCE ============
    m_sim_remove_group = new QGroupBox("Sim Remove Sequence");
    auto *simRemoveLayout = new QVBoxLayout;

    m_sim_remove_btn = new QPushButton("Run Sim Remove Sequence");
    m_sim_remove_btn->setMinimumHeight(45);
    m_sim_remove_btn->setToolTip("Iterate all parts: hover 3cm above, descend, wait 4s (unscrewing), retract. Sim only.");
    simRemoveLayout->addWidget(m_sim_remove_btn);

    m_sim_remove_group->setLayout(simRemoveLayout);
    leftLayout->addWidget(m_sim_remove_group);
```

- [ ] **Step 3: Add connection in setupConnections()**

After the humanoid connections (after line 443), add:

```cpp
    connect(m_sim_remove_btn, &QPushButton::clicked, this, &PoseWindow::runSimRemoveSequence);
```

- [ ] **Step 4: Add the slot implementation**

After `humanoidRandomPose()` (end of file, before the closing of the file), add:

```cpp
void PoseWindow::runSimRemoveSequence()
{
    if (m_sim_remove_running) {
        logStatus("Sim remove sequence already running", true);
        return;
    }

    if (!m_sim_remove_client->wait_for_service(std::chrono::seconds(2))) {
        logStatus("run_sim_remove_sequence service not available — is sim_remove_sequence_node running?", true);
        logBtStatus("Sim remove service not available", true);
        return;
    }

    m_sim_remove_running = true;
    m_sim_remove_btn->setEnabled(false);
    m_sim_remove_btn->setText("Running...");
    logStatus("Starting sim remove sequence (all parts)...");
    logBtStatus("Sim remove sequence started — iterating all parts");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    m_sim_remove_client->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            auto result = future.get();
            QMetaObject::invokeMethod(this, [this, result]() {
                m_sim_remove_running = false;
                m_sim_remove_btn->setEnabled(true);
                m_sim_remove_btn->setText("Run Sim Remove Sequence");

                if (result->success) {
                    logStatus("Sim remove sequence done: " + QString::fromStdString(result->message));
                    logBtStatus("Sim remove: " + QString::fromStdString(result->message));
                } else {
                    logStatus("Sim remove sequence failed: " + QString::fromStdString(result->message), true);
                    logBtStatus("Sim remove failed: " + QString::fromStdString(result->message), true);
                }
            }, Qt::QueuedConnection);
        });
}
```

- [ ] **Step 5: Commit**

```bash
git add src/arpa_gui/src/pose_window.cpp
git commit -m "gui: add Run Sim Remove Sequence button and callback"
```

---

### Task 6: Build and verify

- [ ] **Step 1: Build arpa_helper_tools**

Run: `sudo colcon build --symlink-install --packages-select arpa_helper_tools`

Expected: BUILD SUCCESSFUL, no errors.

- [ ] **Step 2: Build arpa_gui**

Run: `sudo colcon build --symlink-install --packages-select arpa_gui`

Expected: BUILD SUCCESSFUL, no errors.

- [ ] **Step 3: Build arpa_bringup**

Run: `sudo colcon build --symlink-install --packages-select arpa_bringup`

Expected: BUILD SUCCESSFUL, no errors.

- [ ] **Step 4: Verify the node script is installed and executable**

Run: `ls -la install/arpa_helper_tools/lib/arpa_helper_tools/sim_remove_sequence_node.py`

Expected: File exists, is executable (`-rwxr-xr-x` or symlink).

- [ ] **Step 5: Commit any build fixes if needed, then final commit**

```bash
git add -A
git commit -m "build: verify all packages build cleanly with sim remove sequence"
```
