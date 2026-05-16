# Handoff for VLM Pose+Grasp Project — New Device

**Last updated:** 2026-05-17
**Source device:** Laptop A (i7-1165G7, no Isaac Sim, no RealSense, dev only)
**Target device:** Laptop B (RTX 4070 8 GB, **Isaac Sim 4.5**, ROS 2 Humble)
**Branch in flight:** `pose` (16 commits since `c5777f3`, tagged `plan-01-foundation-complete`)
**Goal of project:** Replace nut-runner end effector with parallel gripper, use UR16e+gantry to pick busbars/clamps/etc. from battery table via VLM-driven 6D pose estimation + offline-generated antipodal grasp database.

---

## 1. What's done (Plan 01 — Foundation)

All msg/srv interfaces + the VisionNode publish fix + the PlanToPose mesh-attach hook landed. No new nodes yet. **All work is sim-independent** and built clean on Laptop A.

### Packages touched
- **`custom_ros_messages`** (submodule) — added 4 msgs + 2 srvs:
  - `Detection.msg` (cls/prob/bbox_min/bbox_max + new centroid_world + object_cloud)
  - `DetectionBundle.msg` (rgb_image/depth_image/camera_info/camera_pose/Detection[])
  - `ObjectPose.msg` (label/instance_id/Pose/6x6 covariance/score/mesh_path)
  - `GraspCandidate.msg` (grasp_pose/pre_grasp_pose/width/score/parent_instance_id)
  - `EstimatePose.srv` (label → ObjectPose[])
  - `PlanGrasp.srv` (instance_id/max_attempts → executed GraspCandidate)
  - **Also committed 7 pre-existing untracked msg/srv files** that were referenced by CMakeLists but not in git (would've broken a fresh clone)
- **`arpa_vision`** — `VisionNode.py` populates `Detection.object_cloud` + `centroid_world` in the publish loop; new helper `_o3d_pcd_to_ros` (open3d → PointCloud2 xyz). Imports of `Detection`/`DetectionBundle` work end-to-end now (used to silently fail on missing msg).
- **`arpa_control`** — `PlanToPose.srv` extended with `use_cartesian` + `path_constraints` (moveit_msgs/Constraints) + `mesh_to_attach` + `mesh_to_attach_pose` + `attach_link`. `motion_control_node.cpp` `planToPoseCallback` calls `applyAttachedCollisionObject` when `mesh_to_attach` is non-empty (hard-fails on load/apply failure, defaults `attach_link` to `tool0`).

### What WASN'T done (deferred to Laptop B for runtime)
- **T8 step 5-6**: Launch `VisionNode` + `ros2 topic echo /realsense/ee_cam/detections --once --field detections[0].cls` to verify the bundle actually publishes with the cloud populated. **Requires RealSense camera + workspace bringup.**
- **T10 step 5**: `ros2 run motion_control_node` + `ros2 service call /plan_to_pose ...` with an actual mesh path. **Requires MoveIt + URDF loaded.**

If those tests fail on Laptop B, the source-level fix is local to `VisionNode.py` (cloud population alignment — see §7) and `motion_control_node.cpp` (attach logic — see §5).

---

## 2. First-time setup on Laptop B

```bash
# Clone + submodules
git clone --recurse-submodules git@github.com:correlllab/arpa_ws.git
cd arpa_ws
git checkout pose
git submodule update --init --recursive

# IMPORTANT: build dirs may end up root-owned on first sudo colcon build
# — if so, after first build chown them to your user so subsequent builds don't need sudo:
sudo chown -R "$USER:$USER" build install log

# Build the 3 foundation packages
colcon build --symlink-install --packages-select custom_ros_messages arpa_control arpa_vision
source install/setup.bash

# Verify
python3 -m pytest src/custom_ros_messages/test/ -v   # expect 6 passed
ros2 interface show arpa_control/srv/PlanToPose      # verify new fields present
```

If `colcon build` complains about stale `/root/ros2_ws/build/...` references in `CMakeCache.txt`, just delete the cache for the offending package and rebuild:
```bash
rm -rf build/<pkg> install/<pkg>
colcon build --symlink-install --packages-select <pkg>
```

---

## 3. Plan 02 — Offline CAD tooling (NEXT, on Laptop B)

Standalone Python pipeline: CAD mesh → (a) CoACD convex decomposition for collision, (b) antipodal grasp database, (c) FoundationPose template renders.

### Conda env (recommended — keeps pytorch/cuda isolated from system ROS)
```bash
conda create -n vla-grasp python=3.10 -y
conda activate vla-grasp
pip install coacd pyrender trimesh open3d numpy scipy
# For workspace-IK prefilter (optional): ikpy or pinocchio
```

### Files to create
```
src/arpa_grasp_planning/
├── tools/
│   ├── decompose_mesh.py        # CoACD threshold=0.05, preprocess_resolution=50
│   ├── generate_grasps.py       # antipodal sampler + force closure score
│   ├── render_pose_templates.py # 42 icosahedral views via pyrender
│   └── gripper_spec.yaml        # width_max, width_min, finger_depth, palm_clearance
├── models/                      # output dir: per-class artifacts
│   └── busbar/                  # mesh.obj, collision_decomp.obj, grasps.npy, pose_templates/, metadata.yaml
└── tests/
    └── test_grasp_generator.py  # cube + cylinder unit tests (N grasps, all antipodal, all collision-free)
```

Test on a synthetic cube first, then on the actual busbar CAD. Acceptance criteria: ≥100 grasps per object, at least 1 reachable in nominal table pose, force-closure score >0.5.

Spec reference: `docs/superpowers/specs/2026-05-16-vlm-6dpose-grasp-design.md` §9.

---

## 4. Plan 03 — Pose estimator node (after Plan 02)

ROS 2 wrapper around FoundationPose. Consumes `DetectionBundle` from VisionNode, produces `ObjectPose` per detection.

### FoundationPose setup
```bash
cd src/arpa_pose_estimation/third_party/
git submodule add https://github.com/NVlabs/FoundationPose.git
cd FoundationPose
# Use their conda env (or merge into vla-grasp env)
# Deps: pytorch3d, nvdiffrast, kaolin — finicky, use their dockerfile if possible
bash scripts/download_weights.sh   # ~3-5 GB
```

### Files to create
```
src/arpa_pose_estimation/
├── arpa_pose_estimation/
│   ├── pose_estimator_node.py    # streaming (sub DetectionBundle → pub ObjectPose[]) + on-demand (EstimatePose.srv server)
│   ├── foundationpose_wrapper.py # register + score + track API isolation
│   ├── model_registry.py         # load mesh + templates + metadata from models/<class>/
│   └── icp_fallback.py           # if FP score < 0.3, ICP between depth cloud and mesh
├── package.xml
├── setup.py
└── third_party/FoundationPose/   # submodule
```

### Integration with VisionNode
VisionNode already publishes `/realsense/ee_cam/detections` (DetectionBundle, 6 Hz). `pose_estimator_node` subscribes, crops RGB+depth to padded bbox, runs FoundationPose `register()`, transforms `T_cam_obj` → `T_world_obj` via TF, publishes `ObjectPose[]` on `/object_poses`.

### Instance ID (per spec §7.3)
```python
instance_id = f"{label}_{short_hash(round(centroid_world, 2))}"
```
Stable across consecutive frames so grasp planner can refer to the same physical object.

### VRAM on 4070 8 GB
| Resident | VRAM |
|---|---|
| YOLO_WORLD FP16 (VisionNode) | ~2 GB |
| FoundationPose @ 160×160 crop | ~5 GB |
| **Total** | ~7 GB (1 GB headroom) |

If OOM: run `pose_estimator_node` in a separate process so YOLO can unload while FP runs. Or fall back to SAM6D (spec option B).

### Isaac Sim 4.5 specific notes
- **Camera bridge**: Use `omni.isaac.ros2_bridge` to publish RealSense-equivalent topics. The expected topics are:
  - `/realsense/ee_cam/color/image_raw/compressed`
  - `/realsense/ee_cam/aligned_depth_to_color/image_raw/compressedDepth`
  - `/realsense/ee_cam/color/camera_info`
- **Camera frame**: `ee_cam_color_optical_frame` (per RealSense convention). Make sure the USD prim publishes TF to `world` via the bridge.
- **Sim time vs wall time**: Set `use_sim_time:=true` on VisionNode + pose_estimator_node when running against Isaac. Otherwise TF lookups will fail.
- **Busbar USD**: Convert busbar `.obj` → `.usd` via `omni.kit.tool.asset_importer`. Place at known pose for first pose-accuracy test.
- **Reference**: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/ros2_tutorials/index.html

Acceptance criteria: pose error <5 mm / 5° at 30 cm range, validated against ArUco fiducial OR Isaac Sim ground-truth pose.

Spec reference: `docs/superpowers/specs/2026-05-16-vlm-6dpose-grasp-design.md` §10.

---

## 5. Plan 04 — Grasp planner + BT integration (after Plan 03)

End-to-end pick. Loads grasp DB per class, transforms grasps via `T_world_obj`, filters (collision + IK + manip), executes top-K via existing `PlanToPose` service.

### Files to create
```
src/arpa_grasp_planning/
├── arpa_grasp_planning/
│   ├── grasp_planner_node.py    # PlanGrasp.srv server
│   ├── grasp_filter.py          # IK + collision + scoring
│   └── pick_executor.py         # pre_grasp → grasp → close → retreat-with-attached-mesh
├── ...
src/arpa_bt_executor/
└── ...
└── pick_action_node.py          # BT leaf: calls EstimatePose + PlanGrasp
```

### Stub gripper
Real gripper hardware spec is still TBD. For Plan 04 use stub services that just log:
```python
def stub_close_gripper(self, request, response):
    self.get_logger().info(f"[STUB] CloseGripper(width={request.width})")
    response.success = True
    return response
```
Real gripper driver goes in a **separate spec** triggered once hardware is finalized.

### Acceptance criteria
- Gazebo / Isaac Sim end-to-end: 50 randomized busbar poses → ≥90% pick success
- Re-look retry policy if planner fails all top-K

Spec reference: `docs/superpowers/specs/2026-05-16-vlm-6dpose-grasp-design.md` §11.

---

## 6. Known issues / tech debt

### Server doesn't honor `use_cartesian` or `path_constraints` (yet)
`motion_control_node.cpp::planToPoseCallback` accepts both fields but never reads them. `bt_executor_node.py:249` sets them expecting them to work. There's a TODO comment in `PlanToPose.srv` documenting this. A separate task needs to wire:
- `computeCartesianPath()` branch when `use_cartesian=true`
- `setPathConstraints()` call when `path_constraints` has nonzero size

### Positional cloud-to-bbox alignment in VisionNode
`VisionNode._o3d_pcd_to_ros` pairs tracker cloud at index N with YOLO bbox at index N. Tracker merges duplicates, so the counts can disagree → some Detections get a zero-default cloud. **Plan 03 should replace this with nearest-bbox matching** by projecting tracker centroid into image space. TODO comment is in the source.

### Per-cycle cloud serialization in VisionNode
`_o3d_pcd_to_ros` is called for every tracked detection on every publish cycle (~6 Hz). At >20 detections this becomes the dominant CPU cost. TODO comment recommends caching the serialized PointCloud2 on the `det` dict and invalidating on `_update_detections`. Not blocking at current scale.

### `picked_object` collision-object id is hardcoded
In Plan 04 (sequential picks), two concurrent attached objects would overwrite each other. TODO comment notes this; Plan 04 will move to `f"picked_{instance_id}"`.

### Gripper URDF + driver
`tool_holder.xacro` is still the nut-runner. Replace with `parallel_gripper.xacro` once gripper is finalized. Need: communication protocol, position+force feedback, max force, stroke, finger geometry. Separate spec.

### Other submodule drift (pre-existing, not Plan 01's doing)
At session end on Laptop A, `git status` showed these submodules had drift the parent didn't know about: `Universal_Robots_ROS2_Description`, `Universal_Robots_ROS2_Driver`, `Universal_Robots_ROS2_Gazebo_Simulation`, `arpa_ethernet_motor`, `cl_realsense`, `ur16e_rest`. Some have internal modified content too. Decide on Laptop B whether to bump these pointers or keep them floating.

### Untracked top-level dirs (pre-existing)
`src/g1_description/`, `src/h12_ros2_controller/`, `src/h12_ros2_model/` — these never got added. Probably user WIP humanoid work. Decide whether to commit (or `git submodule add`) before fresh clone.

---

## 7. Workspace gotchas (carry-forward from Laptop A memory)

1. **`build/`, `install/`, `log/` ownership**: previously root-owned on Laptop A. Fix on Laptop B with `sudo chown -R "$USER:$USER" build install log` after first build.
2. **Install symlinks**: `--symlink-install` mode. If symlinks point at `/root/ros2_ws/build/...` (left over from a prior root-owned build), delete the stale `build/<pkg>` + `install/<pkg>` and rebuild.
3. **CycloneDDS required** (FastRTPS crashes with BT node clients). Make sure `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set.
4. **Submodule dual-commit pattern**: when modifying anything in `src/custom_ros_messages/`, commit inside the submodule THEN bump the parent pointer — two commits, in that order.

---

## 8. Documents to read on Laptop B

| File | Purpose |
|---|---|
| `docs/superpowers/specs/2026-05-16-vlm-6dpose-grasp-design.md` | Full design spec — architecture, msgs, offline pipeline, performance budget, risks |
| `docs/superpowers/plans/2026-05-16-vlm-pose-grasp-roadmap.md` | Plan 02-04 outlines |
| `docs/superpowers/plans/2026-05-16-vlm-pose-grasp-plan-01-foundation.md` | Bite-sized Plan 01 (already executed — read for context if debugging foundation code) |

---

## 9. Suggested order on Laptop B

1. Clone, build foundation (per §2). Verify 6 tests pass + `ros2 interface show` works.
2. Run Plan 01 deferred runtime checks (T8 step 5-6 + T10 step 5) against Isaac Sim 4.5 scene with at least one detected object (e.g., busbar USD on table).
3. Brainstorm + write Plan 02 in detail (use `superpowers:brainstorming` then `superpowers:writing-plans`). Execute via `superpowers:subagent-driven-development`.
4. Brainstorm + write Plan 03. Execute. Validate pose error <5 mm / 5°.
5. Brainstorm + write Plan 04. Execute. Validate ≥90% pick success.
6. When gripper hardware is finalized, brand new spec + plan for hardware bring-up.

---

## 10. Commit attribution audit

Of 24 commits on `pose` (16 parent + 8 submodule), **only 4 include the `Co-Authored-By: Claude` trailer**. The rest were made by subagents whose commit messages I wrote without the trailer. If consistent attribution matters, before pushing run an interactive rebase to amend each commit body — but this rewrites SHAs.

---

## 11. Misc context

- **Date today on Laptop A:** 2026-05-17
- **User email on Laptop A:** allendevaraj33333@gmail.com
- **Origin remote:** `git@github.com:correlllab/arpa_ws.git`
- **Branch in flight:** `pose`
- **Tag at HEAD:** `plan-01-foundation-complete` (will need to be pushed: `git push origin plan-01-foundation-complete`)
