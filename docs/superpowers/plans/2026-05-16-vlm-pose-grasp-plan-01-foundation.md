# VLM Pose+Grasp — Plan 01: Msg/Srv Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land all ROS 2 message + service definitions that the VLM 6D pose + grasp pipeline depends on, fix the broken `DetectionBundle` publisher in `VisionNode`, and extend `PlanToPose.srv` so picked objects become attached collision objects after grasp.

**Architecture:** Additive changes to `custom_ros_messages` and `arpa_control` interface packages, plus a populate-and-publish fix to the existing `arpa_vision/VisionNode.py`. No new nodes. Field names match the existing `VisionNode` usage so it works as-is once messages exist.

**Tech Stack:** ROS 2 Humble, `rosidl` (CMake), Python 3.10, `colcon build --symlink-install`.

**Source spec:** `docs/superpowers/specs/2026-05-16-vlm-6dpose-grasp-design.md` §7. Field names in this plan use the existing `VisionNode.py` names (`cls`, `bbox_min`, `bbox_max`, `prob`, `rgb_image`, `depth_image`) — supersedes spec §7.1/7.2.

**Workspace gotchas (per `~/.claude/.../MEMORY.md`):**
- Build/install/log directories are **root-owned** — every `colcon build` needs `sudo`.
- Install dir uses `--symlink-install`; existing symlinks may point to non-existent `/root/ros2_ws/build/`. After a successful build, verify with `ls -la install/<pkg>/lib/<pkg>/`. If broken, the build itself fixes them when run with `--symlink-install`.

---

### Task 1: Add `Detection.msg`

**Files:**
- Create: `src/custom_ros_messages/msg/Detection.msg`
- Modify: `src/custom_ros_messages/CMakeLists.txt`
- Test: `src/custom_ros_messages/test/test_new_msgs.py`

- [ ] **Step 1: Write failing test**

Create `src/custom_ros_messages/test/test_new_msgs.py`:

```python
import pytest


def test_detection_constructs_with_existing_visionnode_fields():
    from custom_ros_messages.msg import Detection
    from geometry_msgs.msg import Point
    d = Detection()
    d.cls = "BusBar"
    d.prob = 0.95
    d.bbox_min = Point(x=10.0, y=20.0, z=0.0)
    d.bbox_max = Point(x=110.0, y=220.0, z=0.0)
    d.centroid_world = Point(x=0.5, y=0.6, z=0.1)
    # object_cloud is sensor_msgs/PointCloud2; default-constructable empty cloud is fine
    assert d.cls == "BusBar"
    assert d.bbox_max.x == 110.0
```

- [ ] **Step 2: Run test, expect ImportError**

```bash
cd /home/the2xman/arpa_ws
python -c "from custom_ros_messages.msg import Detection"
```

Expected: `ImportError: cannot import name 'Detection'`.

- [ ] **Step 3: Create the message file**

Create `src/custom_ros_messages/msg/Detection.msg`:

```
string cls
float32 prob
geometry_msgs/Point bbox_min
geometry_msgs/Point bbox_max
geometry_msgs/Point centroid_world
sensor_msgs/PointCloud2 object_cloud
```

- [ ] **Step 4: Register in CMakeLists.txt**

Open `src/custom_ros_messages/CMakeLists.txt`. Find the `rosidl_generate_interfaces` block. Insert `"msg/Detection.msg"` between `"msg/DetectedParts.msg"` and `"srv/Query.srv"`:

```cmake
  "msg/DetectedParts.msg"
  "msg/Detection.msg"
  "srv/Query.srv"
```

- [ ] **Step 5: Build the package**

```bash
cd /home/the2xman/arpa_ws
sudo colcon build --symlink-install --packages-select custom_ros_messages
```

Expected: `Finished <<< custom_ros_messages` with no errors. If `Permission denied`, the build dir is root-owned — confirm `sudo` was used.

- [ ] **Step 6: Run the test**

```bash
cd /home/the2xman/arpa_ws
source install/setup.bash
python -m pytest src/custom_ros_messages/test/test_new_msgs.py::test_detection_constructs_with_existing_visionnode_fields -v
```

Expected: 1 passed.

- [ ] **Step 7: Commit**

```bash
git add src/custom_ros_messages/msg/Detection.msg src/custom_ros_messages/CMakeLists.txt src/custom_ros_messages/test/test_new_msgs.py
git commit -m "feat(msgs): add Detection.msg matching existing VisionNode field names"
```

---

### Task 2: Add `DetectionBundle.msg`

**Files:**
- Create: `src/custom_ros_messages/msg/DetectionBundle.msg`
- Modify: `src/custom_ros_messages/CMakeLists.txt`
- Modify: `src/custom_ros_messages/test/test_new_msgs.py` (append test)

- [ ] **Step 1: Append failing test**

Append to `src/custom_ros_messages/test/test_new_msgs.py`:

```python
def test_detection_bundle_constructs_with_existing_visionnode_fields():
    from custom_ros_messages.msg import DetectionBundle, Detection
    b = DetectionBundle()
    b.detections = [Detection(), Detection()]
    assert len(b.detections) == 2
    # Field name checks (matches VisionNode usage at line ~392):
    assert hasattr(b, "rgb_image")
    assert hasattr(b, "depth_image")
    assert hasattr(b, "camera_info")
    assert hasattr(b, "camera_pose")
```

- [ ] **Step 2: Run test, expect ImportError**

```bash
python -c "from custom_ros_messages.msg import DetectionBundle"
```

Expected: `ImportError`.

- [ ] **Step 3: Create message file**

Create `src/custom_ros_messages/msg/DetectionBundle.msg`:

```
std_msgs/Header header
sensor_msgs/CompressedImage rgb_image
sensor_msgs/CompressedImage depth_image
sensor_msgs/CameraInfo camera_info
geometry_msgs/PoseStamped camera_pose
Detection[] detections
```

- [ ] **Step 4: Register in CMakeLists.txt**

Insert `"msg/DetectionBundle.msg"` directly after the `"msg/Detection.msg"` line added in Task 1:

```cmake
  "msg/Detection.msg"
  "msg/DetectionBundle.msg"
  "srv/Query.srv"
```

- [ ] **Step 5: Build**

```bash
sudo colcon build --symlink-install --packages-select custom_ros_messages
```

Expected: `Finished <<< custom_ros_messages`.

- [ ] **Step 6: Run both tests**

```bash
source install/setup.bash
python -m pytest src/custom_ros_messages/test/test_new_msgs.py -v
```

Expected: 2 passed.

- [ ] **Step 7: Commit**

```bash
git add src/custom_ros_messages/msg/DetectionBundle.msg src/custom_ros_messages/CMakeLists.txt src/custom_ros_messages/test/test_new_msgs.py
git commit -m "feat(msgs): add DetectionBundle.msg matching existing VisionNode field names"
```

---

### Task 3: Add `ObjectPose.msg`

**Files:**
- Create: `src/custom_ros_messages/msg/ObjectPose.msg`
- Modify: `src/custom_ros_messages/CMakeLists.txt`
- Modify: `src/custom_ros_messages/test/test_new_msgs.py`

- [ ] **Step 1: Append failing test**

```python
def test_object_pose_msg_fields():
    from custom_ros_messages.msg import ObjectPose
    op = ObjectPose()
    op.label = "BusBar"
    op.instance_id = "BusBar_a1b2c3"
    op.score = 0.87
    op.mesh_path = "/abs/path/models/busbar/mesh.obj"
    op.covariance = [0.0] * 36
    assert op.label == "BusBar"
    assert len(op.covariance) == 36
```

- [ ] **Step 2: Run, expect ImportError**

```bash
python -c "from custom_ros_messages.msg import ObjectPose"
```

- [ ] **Step 3: Create file**

Create `src/custom_ros_messages/msg/ObjectPose.msg`:

```
std_msgs/Header header
string label
string instance_id
geometry_msgs/Pose pose
float32[36] covariance
float32 score
string mesh_path
```

- [ ] **Step 4: Register in CMakeLists**

Insert after `"msg/DetectionBundle.msg"`:

```cmake
  "msg/DetectionBundle.msg"
  "msg/ObjectPose.msg"
  "srv/Query.srv"
```

- [ ] **Step 5: Build**

```bash
sudo colcon build --symlink-install --packages-select custom_ros_messages
```

- [ ] **Step 6: Run tests**

```bash
source install/setup.bash
python -m pytest src/custom_ros_messages/test/test_new_msgs.py -v
```

Expected: 3 passed.

- [ ] **Step 7: Commit**

```bash
git add src/custom_ros_messages/msg/ObjectPose.msg src/custom_ros_messages/CMakeLists.txt src/custom_ros_messages/test/test_new_msgs.py
git commit -m "feat(msgs): add ObjectPose.msg for 6D pose pipeline output"
```

---

### Task 4: Add `GraspCandidate.msg`

**Files:**
- Create: `src/custom_ros_messages/msg/GraspCandidate.msg`
- Modify: `src/custom_ros_messages/CMakeLists.txt`
- Modify: `src/custom_ros_messages/test/test_new_msgs.py`

- [ ] **Step 1: Append test**

```python
def test_grasp_candidate_msg_fields():
    from custom_ros_messages.msg import GraspCandidate
    gc = GraspCandidate()
    gc.width = 0.04
    gc.score = 0.72
    gc.parent_instance_id = "BusBar_a1b2c3"
    assert 0 < gc.width < 1
```

- [ ] **Step 2: Run, expect ImportError**

```bash
python -c "from custom_ros_messages.msg import GraspCandidate"
```

- [ ] **Step 3: Create file**

Create `src/custom_ros_messages/msg/GraspCandidate.msg`:

```
geometry_msgs/Pose grasp_pose
geometry_msgs/Pose pre_grasp_pose
float32 width
float32 score
string parent_instance_id
```

- [ ] **Step 4: Register in CMakeLists**

Insert after `"msg/ObjectPose.msg"`:

```cmake
  "msg/ObjectPose.msg"
  "msg/GraspCandidate.msg"
  "srv/Query.srv"
```

- [ ] **Step 5: Build**

```bash
sudo colcon build --symlink-install --packages-select custom_ros_messages
```

- [ ] **Step 6: Run tests**

```bash
source install/setup.bash
python -m pytest src/custom_ros_messages/test/test_new_msgs.py -v
```

Expected: 4 passed.

- [ ] **Step 7: Commit**

```bash
git add src/custom_ros_messages/msg/GraspCandidate.msg src/custom_ros_messages/CMakeLists.txt src/custom_ros_messages/test/test_new_msgs.py
git commit -m "feat(msgs): add GraspCandidate.msg for grasp planner output"
```

---

### Task 5: Add `EstimatePose.srv`

**Files:**
- Create: `src/custom_ros_messages/srv/EstimatePose.srv`
- Modify: `src/custom_ros_messages/CMakeLists.txt`
- Modify: `src/custom_ros_messages/test/test_new_srvs.py` (new file)

- [ ] **Step 1: Create test file**

Create `src/custom_ros_messages/test/test_new_srvs.py`:

```python
def test_estimate_pose_srv_constructs():
    from custom_ros_messages.srv import EstimatePose
    req = EstimatePose.Request()
    req.label = "BusBar"
    resp = EstimatePose.Response()
    resp.success = True
    resp.message = "ok"
    assert req.label == "BusBar"
    assert resp.success is True
```

- [ ] **Step 2: Run, expect ImportError**

```bash
python -c "from custom_ros_messages.srv import EstimatePose"
```

- [ ] **Step 3: Create srv file**

Create `src/custom_ros_messages/srv/EstimatePose.srv`:

```
string label
---
ObjectPose[] poses
bool success
string message
```

- [ ] **Step 4: Register in CMakeLists**

Find the srv block and insert `"srv/EstimatePose.srv"` after the last srv entry (before `action/`):

```cmake
  "srv/RemovePart.srv"
  "srv/EstimatePose.srv"
  "action/DualArm.action"
```

- [ ] **Step 5: Build**

```bash
sudo colcon build --symlink-install --packages-select custom_ros_messages
```

Expected: `Finished <<< custom_ros_messages`. If a build error mentions `ObjectPose not found`, Task 3 was not built — re-run Task 3 step 5.

- [ ] **Step 6: Run tests**

```bash
source install/setup.bash
python -m pytest src/custom_ros_messages/test/test_new_srvs.py -v
```

Expected: 1 passed.

- [ ] **Step 7: Commit**

```bash
git add src/custom_ros_messages/srv/EstimatePose.srv src/custom_ros_messages/CMakeLists.txt src/custom_ros_messages/test/test_new_srvs.py
git commit -m "feat(srvs): add EstimatePose.srv for on-demand 6D pose"
```

---

### Task 6: Add `PlanGrasp.srv`

**Files:**
- Create: `src/custom_ros_messages/srv/PlanGrasp.srv`
- Modify: `src/custom_ros_messages/CMakeLists.txt`
- Modify: `src/custom_ros_messages/test/test_new_srvs.py`

- [ ] **Step 1: Append test**

```python
def test_plan_grasp_srv_constructs():
    from custom_ros_messages.srv import PlanGrasp
    req = PlanGrasp.Request()
    req.instance_id = "BusBar_a1b2c3"
    req.max_attempts = 5
    resp = PlanGrasp.Response()
    resp.success = False
    resp.message = "ik failure"
    assert req.max_attempts == 5
    assert resp.success is False
```

- [ ] **Step 2: Run, expect ImportError**

```bash
python -c "from custom_ros_messages.srv import PlanGrasp"
```

- [ ] **Step 3: Create srv file**

Create `src/custom_ros_messages/srv/PlanGrasp.srv`:

```
string instance_id
uint8 max_attempts
---
GraspCandidate executed
bool success
string message
```

- [ ] **Step 4: Register in CMakeLists**

Insert after `"srv/EstimatePose.srv"`:

```cmake
  "srv/EstimatePose.srv"
  "srv/PlanGrasp.srv"
  "action/DualArm.action"
```

- [ ] **Step 5: Build**

```bash
sudo colcon build --symlink-install --packages-select custom_ros_messages
```

- [ ] **Step 6: Run tests**

```bash
source install/setup.bash
python -m pytest src/custom_ros_messages/test/ -v
```

Expected: 6 passed (4 msg tests + 2 srv tests).

- [ ] **Step 7: Commit**

```bash
git add src/custom_ros_messages/srv/PlanGrasp.srv src/custom_ros_messages/CMakeLists.txt src/custom_ros_messages/test/test_new_srvs.py
git commit -m "feat(srvs): add PlanGrasp.srv for grasp planner entry point"
```

---

### Task 7: Verify install symlinks are not broken

**Files:** none modified; verification only.

This is the workspace gotcha noted in `MEMORY.md` — symlinks can point to `/root/ros2_ws/build/`. Catch it once now while the package was just rebuilt.

- [ ] **Step 1: List install symlinks**

```bash
cd /home/the2xman/arpa_ws
ls -la install/custom_ros_messages/lib/python3.10/site-packages/custom_ros_messages/msg/ | head -20
```

Expected: symlinks pointing to `/home/the2xman/arpa_ws/build/custom_ros_messages/...`. **NOT** `/root/ros2_ws/...`.

- [ ] **Step 2: If broken, force rebuild**

If any link points to `/root/...`, run:

```bash
sudo rm -rf install/custom_ros_messages build/custom_ros_messages
sudo colcon build --symlink-install --packages-select custom_ros_messages
```

Re-run step 1 to confirm.

- [ ] **Step 3: Sanity-import everything from a fresh shell**

```bash
bash -c "source /home/the2xman/arpa_ws/install/setup.bash && python -c '
from custom_ros_messages.msg import Detection, DetectionBundle, ObjectPose, GraspCandidate
from custom_ros_messages.srv import EstimatePose, PlanGrasp
print(\"all imports OK\")
'"
```

Expected: `all imports OK`.

- [ ] **Step 4: No commit needed** (verification step only).

---

### Task 8: Fix `VisionNode` populate-and-publish loop

The publish code at `src/arpa_vision/arpa_vision/scripts/VisionNode.py:390-407` already exists but never includes the per-detection point cloud or world centroid in the `Detection`. Add them.

**Files:**
- Modify: `src/arpa_vision/arpa_vision/scripts/VisionNode.py` (lines ~390-407)
- Test: manual launch + `ros2 topic echo`

- [ ] **Step 1: Read existing publish block**

Open `src/arpa_vision/arpa_vision/scripts/VisionNode.py` and locate the `# DetectionBundle` block (around line 390). Note that `self.detections` is the persistent dict of `{label: [{'pcd':..., 'bbox':..., 'prob':...}]}` while `self.latest_candidates` is the YOLO-frame candidates.

- [ ] **Step 2: Replace the publish block**

Replace the block from `bundle = DetectionBundle()` through `self.detection_pub.publish(bundle)` with:

```python
            bundle = DetectionBundle()
            bundle.header = self.last_rgb_msg.header
            bundle.rgb_image = self.last_rgb_msg
            bundle.depth_image = self.last_depth_msg
            bundle.camera_info = self.last_info_msg
            bundle.camera_pose = self.last_camera_pose

            # Build a lookup of (label, bbox_tuple) -> world cloud + centroid from persistent detections.
            cloud_by_key = {}
            for label, dets in detections.items():
                for det in dets:
                    bbox = det['bbox']  # open3d AxisAlignedBoundingBox in world frame
                    centroid = bbox.get_center().numpy()
                    pcd = det['pcd']
                    # Convert open3d tensor PCD -> sensor_msgs/PointCloud2 in 'world' frame
                    cloud_msg = self._o3d_pcd_to_ros(pcd, frame_id=BASE_FRAME, stamp=self.last_rgb_msg.header.stamp)
                    cloud_by_key.setdefault(label, []).append((cloud_msg, centroid))

            for label, pred in self.latest_candidates.items():
                clouds_for_label = cloud_by_key.get(label, [])
                for idx, (box, prob) in enumerate(zip(pred['boxes'], pred['probs'])):
                    x1, y1, x2, y2 = map(int, box)
                    d = Detection()
                    d.cls = label
                    d.bbox_min = Point(x=float(x1), y=float(y1), z=0.0)
                    d.bbox_max = Point(x=float(x2), y=float(y2), z=0.0)
                    d.prob = float(prob)
                    if idx < len(clouds_for_label):
                        cloud_msg, centroid = clouds_for_label[idx]
                        d.object_cloud = cloud_msg
                        d.centroid_world = Point(x=float(centroid[0]), y=float(centroid[1]), z=float(centroid[2]))
                    bundle.detections.append(d)
            self.detection_pub.publish(bundle)
```

- [ ] **Step 3: Add the `_o3d_pcd_to_ros` helper**

At the bottom of the `VisionNode` class (before `def main()` if present, otherwise before the final closing of the class), add:

```python
    def _o3d_pcd_to_ros(self, pcd, frame_id, stamp):
        """Convert an open3d.t.geometry.PointCloud to sensor_msgs/PointCloud2 (xyz only)."""
        import struct
        pts = pcd.point['positions'].numpy().astype(np.float32)
        cloud = PointCloud2()
        cloud.header.frame_id = frame_id
        cloud.header.stamp = stamp
        cloud.height = 1
        cloud.width = int(pts.shape[0])
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.data = pts.tobytes()
        return cloud
```

- [ ] **Step 4: Build the vision package**

```bash
sudo colcon build --symlink-install --packages-select arpa_vision
```

Expected: `Finished <<< arpa_vision`.

- [ ] **Step 5: Launch the node and verify the topic appears**

In one terminal:

```bash
source /home/the2xman/arpa_ws/install/setup.bash
ros2 run arpa_vision VisionNode 2>&1 | tee /tmp/vision_node.log
```

In a second terminal (after the node is running and detecting at least one object):

```bash
source /home/the2xman/arpa_ws/install/setup.bash
ros2 topic info /realsense/ee_cam/detections
ros2 topic echo /realsense/ee_cam/detections --once --field detections[0].cls
```

Expected: topic type `custom_ros_messages/msg/DetectionBundle`, echo prints a label string.

- [ ] **Step 6: Verify cloud is populated**

```bash
ros2 topic echo /realsense/ee_cam/detections --once --field detections[0].object_cloud.width
```

Expected: a positive integer (e.g. `523`). If `0`, the lookup index alignment is off — re-read step 2 carefully.

- [ ] **Step 7: Commit**

```bash
git add src/arpa_vision/arpa_vision/scripts/VisionNode.py
git commit -m "fix(vision): populate Detection.object_cloud and centroid_world in DetectionBundle"
```

---

### Task 9: Extend `PlanToPose.srv` with attached-mesh fields

After a successful grasp, the planner needs to tell MoveIt to attach the picked mesh to the gripper as a collision object so the retreat motion knows about it. Add optional request fields.

**Files:**
- Modify: `src/arpa_control/srv/PlanToPose.srv`
- Modify: `src/arpa_control/src/motion_control_node.cpp` (consumers — verify backward-compat)
- Test: build verifies; behavioural test via grasp planner in Plan 04.

- [ ] **Step 1: Note current srv content**

Current file:

```
geometry_msgs/PoseStamped target_pose
---
bool success
string message
```

- [ ] **Step 2: Add new fields**

Replace contents of `src/arpa_control/srv/PlanToPose.srv` with:

```
geometry_msgs/PoseStamped target_pose
bool use_cartesian
string path_constraints
string mesh_to_attach
geometry_msgs/Pose mesh_to_attach_pose
string attach_link
---
bool success
string message
```

Notes for the engineer:
- `use_cartesian` and `path_constraints` are listed in your memory as already existing in this srv — if they are not currently in the file, this commit adds them (they should be no-ops if motion_control_node ignores them).
- `mesh_to_attach` empty string = no-op (backward compatible with existing callers).
- `attach_link` empty string = default to `tool0`.

- [ ] **Step 3: Build the control package**

```bash
sudo colcon build --symlink-install --packages-select arpa_control
```

If `motion_control_node.cpp` fails to compile due to missing field references it had before (`use_cartesian`, `path_constraints`), the srv was already in the older form — restore those fields and re-run.

Expected on success: `Finished <<< arpa_control`.

- [ ] **Step 4: Smoke-verify the node still launches**

```bash
source /home/the2xman/arpa_ws/install/setup.bash
ros2 run arpa_control motion_control_node &
sleep 3
ros2 service list | grep plan_to_pose
ros2 service type /plan_to_pose
kill %1
```

Expected: `/plan_to_pose` listed with type `arpa_control/srv/PlanToPose`.

- [ ] **Step 5: Confirm existing callers still work**

Search for callers across the workspace:

```bash
grep -rn "plan_to_pose" src/ --include="*.py" --include="*.cpp" --include="*.hpp" | grep -v "^Binary" | grep -v "_ws/build" | head -20
```

For each caller file, confirm it does not error on the new optional fields (Python: default-constructed `Request()` zeroes everything; C++: same). If a caller fails to compile after this commit, leave the new fields blank in that caller — defaults are no-op.

- [ ] **Step 6: Commit**

```bash
git add src/arpa_control/srv/PlanToPose.srv
git commit -m "feat(control): extend PlanToPose.srv with attached-mesh fields"
```

---

### Task 10: Implement attached-mesh handling in `motion_control_node.cpp`

The new request fields are no-ops until the C++ service callback uses them. Implement minimal handling so that when `mesh_to_attach` is non-empty, the planner attaches the mesh to `attach_link` (default `tool0`) before executing.

**Files:**
- Modify: `src/arpa_control/src/motion_control_node.cpp` (around line 481, the `plan_to_pose` callback)

- [ ] **Step 1: Read current callback signature**

```bash
sed -n '475,540p' src/arpa_control/src/motion_control_node.cpp
```

Identify where the callback receives `request` and where it builds the goal. Locate where it returns success.

- [ ] **Step 2: Add helper function for attaching meshes**

Near the top of `motion_control_node.cpp`, after existing includes, add:

```cpp
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
```

If `geometric_shapes` is not yet in `CMakeLists.txt` / `package.xml`, add it.

`CMakeLists.txt`:
```cmake
find_package(geometric_shapes REQUIRED)
```
and append `geometric_shapes` to the `ament_target_dependencies(motion_control_node ...)` block.

`package.xml`:
```xml
<depend>geometric_shapes</depend>
```

Then add the helper function above the class:

```cpp
static moveit_msgs::msg::AttachedCollisionObject build_attached_mesh(
    const std::string & mesh_path,
    const geometry_msgs::msg::Pose & mesh_pose,
    const std::string & link_name)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = link_name;
  aco.object.id = "picked_object";
  aco.object.header.frame_id = link_name;
  aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shapes::Mesh * m = shapes::createMeshFromResource("file://" + mesh_path);
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(m, shape_msg);
  aco.object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
  aco.object.mesh_poses.push_back(mesh_pose);
  delete m;
  return aco;
}
```

- [ ] **Step 3: Use the helper in the callback**

Inside the `plan_to_pose` callback (around line 481-530), immediately after the request is received and before planning starts, add:

```cpp
  if (!request->mesh_to_attach.empty()) {
    std::string link = request->attach_link.empty() ? std::string("tool0") : request->attach_link;
    auto aco = build_attached_mesh(request->mesh_to_attach, request->mesh_to_attach_pose, link);
    auto psi = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    psi->applyAttachedCollisionObject(aco);
    RCLCPP_INFO(this->get_logger(), "Attached mesh '%s' to link '%s'", request->mesh_to_attach.c_str(), link.c_str());
  }
```

If `psi` already exists as a class member in the node, use that instance instead of constructing a new one.

- [ ] **Step 4: Build**

```bash
sudo colcon build --symlink-install --packages-select arpa_control
```

Expected: `Finished <<< arpa_control`. Compile errors most likely indicate missing `geometric_shapes` dependency (re-check step 2 dep edits).

- [ ] **Step 5: Verify backward compat — call service with empty mesh field**

```bash
source /home/the2xman/arpa_ws/install/setup.bash
ros2 run arpa_control motion_control_node &
sleep 5
ros2 service call /plan_to_pose arpa_control/srv/PlanToPose "{target_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: 0.5, z: 0.4}, orientation: {w: 1.0}}}}"
kill %1
```

Expected: service responds (success or planning failure — either is fine; the point is it does not crash on the new fields being empty).

- [ ] **Step 6: Commit**

```bash
git add src/arpa_control/src/motion_control_node.cpp src/arpa_control/CMakeLists.txt src/arpa_control/package.xml
git commit -m "feat(control): handle mesh_to_attach in plan_to_pose callback"
```

---

### Task 11: End-of-plan integration check

**Files:** none modified.

- [ ] **Step 1: Full workspace build**

```bash
cd /home/the2xman/arpa_ws
sudo colcon build --symlink-install --packages-select custom_ros_messages arpa_vision arpa_control
```

Expected: 3 packages finish without errors.

- [ ] **Step 2: All tests pass**

```bash
source install/setup.bash
python -m pytest src/custom_ros_messages/test/ -v
```

Expected: 6 passed.

- [ ] **Step 3: Mark plan complete**

This plan ends here. Plan 02 (offline CAD tooling) and Plan 03 (pose estimator node) can now build on these msgs/srvs.

- [ ] **Step 4: Optional tag**

```bash
git tag plan-01-foundation-complete
```
