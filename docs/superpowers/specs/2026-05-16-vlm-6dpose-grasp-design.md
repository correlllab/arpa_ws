# VLM-Driven 6D Pose + Grasp Planning Pipeline — Design

**Date:** 2026-05-16
**Status:** Draft, awaiting implementation plan
**Author:** brainstorming session with Claude

## 1. Goal

Replace the current nut-runner end effector with a parallel-jaw gripper and use the existing UR16e+gantry stack to **pick objects** (busbars, clamps, etc.) from the battery table. The pipeline consumes RGB+depth from the existing eye-in-hand RealSense (`ee_cam`), produces 6D object poses for objects identified by the already-deployed YOLO_WORLD detector, and generates collision-aware grasp poses suitable for MoveIt execution. CAD models exist (or will exist) for all target objects.

## 2. Non-goals

- **Place** planning (separate spec, future).
- Multi-arm coordination.
- Soft object handling.
- Replacing the existing YOLO_WORLD 2D detector (it stays, feeds new pipeline).
- The actual gripper hardware bring-up (driver + URDF replacement covered in a separate spec once gripper model is finalized).

## 3. Hardware / software context

| Item | Value |
|---|---|
| Robot | UR16e on 1-DOF linear gantry (7-DOF total) |
| Camera | RealSense (eye-in-hand on `ee_cam`, serial `838212073340`) |
| Compute | RTX 4070 8 GB VRAM, i7-1165G7, 16 GB RAM |
| OS | Ubuntu 22.04 + ROS 2 Humble + MoveIt 2 + CycloneDDS |
| Existing detector | YOLO_WORLD (yolov8x-worldv2.pt, FP16) in `arpa_vision` |
| Existing planning | `arpa_control` services: `PlanToPose`, `PlanToJoint`, `ExecutePlan`, `PlanLinearActuator` |
| Existing BT | `arpa_bt_executor` orchestrates sequences |

## 4. Architecture

```
                    OFFLINE (per new CAD)                                     ONLINE
   ┌──────────────────────────────────────────────┐    ┌──────────────────────────────────────────────────┐
   │ CAD .obj/.stl                                │    │ RealSense ee_cam (RGB+Depth+CamInfo @ 6Hz)       │
   │  ├─→ CoACD ─→ convex hulls (collision mesh) │    │   │                                              │
   │  ├─→ Antipodal grasp sampler ─→ Grasp DB    │    │   ▼                                              │
   │  └─→ Pose template renderer ─→ templates    │    │ VisionNode (existing) ─→ YOLO_WORLD              │
   │                                              │    │   │  per-object PointCloud2 in world frame      │
   │ Output: models/<class>/{mesh.obj,            │    │   │  publishes DetectionBundle                   │
   │   collision_decomp.obj, grasps.npy,          │    │   ▼                                              │
   │   pose_templates/, metadata.yaml}            │    │ PoseEstimatorNode (NEW)                          │
   └──────────────────────────────────────────────┘    │   • crop RGB+depth to bbox                       │
                                                       │   • FoundationPose register → T_cam_obj          │
                                                       │   • TF to world → T_world_obj                    │
                                                       │   • Publishes ObjectPose (cov, score, mesh_path) │
                                                       │   ▼                                              │
                                                       │ GraspPlannerNode (NEW)                           │
                                                       │   • Loads grasp DB per class                     │
                                                       │   • Transforms grasps to world                   │
                                                       │   • Filters: collision (CoACD) + IK + manip      │
                                                       │   • Calls PlanToPose top-K w/ fallback chain     │
                                                       │   ▼                                              │
                                                       │ MoveIt (existing) → executes pick                │
                                                       └──────────────────────────────────────────────────┘
```

### 4.1 Approach choice rationale

| Approach | Verdict |
|---|---|
| **A. FoundationPose + offline antipodal grasps + CoACD** (CHOSEN) | Best leverages "have CAD"; deterministic grasps that can be IK-prefiltered; novel-object capable so no retrain when parts added. |
| B. SAM6D + same grasp DB | Lower VRAM, simpler — fallback option if FoundationPose VRAM doesn't fit. |
| C. Contact-GraspNet (model-free) | Wastes CAD asset; no class-level reasoning; harder to make grasps IK-stable. |

## 5. Components

### 5.1 New package: `arpa_pose_estimation`

| File | Purpose |
|---|---|
| `pose_estimator_node.py` | Two modes: (1) **streaming** — subscribes to `DetectionBundle` topic, runs FoundationPose on every bundle, publishes `ObjectPose[]` on `/object_poses` topic; (2) **on-demand** — serves `EstimatePose.srv` for one-shot blocking calls from BT/GUI. Both share the same inference path. |
| `model_registry.py` | Loads CAD mesh, pose templates, grasp DB per class at startup |
| `foundationpose_wrapper.py` | Thin wrapper isolating upstream NVlabs/FoundationPose imports |
| `third_party/FoundationPose/` | Git submodule |
| `setup.sh` | Conda env + weight download |

### 5.2 New package: `arpa_grasp_planning`

| File | Purpose |
|---|---|
| `grasp_planner_node.py` | Receives `ObjectPose`, serves `PlanGrasp.srv`, calls MoveIt |
| `grasp_filter.py` | IK + collision + manipulability scoring |
| `tools/generate_grasps.py` | OFFLINE: mesh + gripper spec → grasps.npy |
| `tools/decompose_mesh.py` | OFFLINE: mesh → CoACD convex hulls |
| `tools/render_pose_templates.py` | OFFLINE: mesh → FoundationPose template renders |
| `models/<class>/` | Per-class artifacts (see §6) |

### 5.3 Modified packages

| Package | Change |
|---|---|
| `custom_ros_messages` | Add `DetectionBundle.msg`, `Detection.msg`, `ObjectPose.msg`, `GraspCandidate.msg`; add `EstimatePose.srv`, `PlanGrasp.srv` |
| `arpa_vision/VisionNode.py` | Fix broken `DetectionBundle` reference (msg not in CMakeLists today); publish bundle on `detection_topic` |
| `arpa_description` | Replace `tool_holder.xacro` with `parallel_gripper.xacro` (after gripper finalized — separate ticket) |
| `arpa_moveit_config` | Update SRDF end-effector group to gripper |
| `arpa_control/PlanToPose.srv` | Add `string mesh_to_attach`, `geometry_msgs/Pose mesh_to_attach_pose` so picked object becomes attached collision object after grasp |
| `arpa_bt_executor` | Add `Pick` action node calling EstimatePose + PlanGrasp |

## 6. Offline artifact layout

```
arpa_grasp_planning/models/
  busbar/
    mesh.obj                # source CAD, scaled meters
    collision_decomp.obj    # CoACD convex hulls, ≤32 parts
    grasps.npy              # structured array: pose_4x4, width_m, score, force_closure
    pose_templates/         # 42 .npz: depth + normal renders, icosahedral viewpoints
    metadata.yaml           # {mass_kg, friction_mu, class_name, version, gripper_compat: [parallel_v1]}
  clamp/ ...
  ...
```

## 7. Message + service definitions

### 7.1 `DetectionBundle.msg`
```
std_msgs/Header header
sensor_msgs/CompressedImage rgb
sensor_msgs/CompressedImage depth
sensor_msgs/CameraInfo camera_info
geometry_msgs/PoseStamped camera_pose
Detection[] detections
```

### 7.2 `Detection.msg`
```
string label
float32 confidence
uint16[4] bbox_xyxy
sensor_msgs/PointCloud2 object_cloud
geometry_msgs/Point centroid_world
```

### 7.3 `ObjectPose.msg`
```
std_msgs/Header header
string label
string instance_id          # stable id assigned by pose_estimator_node: "<label>_<short_hash(initial_centroid_world)>"
geometry_msgs/Pose pose
float32[36] covariance
float32 score               # FoundationPose refiner score in [0, 1], higher = better
string mesh_path
```
`instance_id` is generated on first observation by hashing the initial centroid (rounded to 1 cm) so the same physical object yields the same id across consecutive frames; the `model_registry` maintains the mapping until the object leaves the scene or is picked.

### 7.4 `GraspCandidate.msg`
```
geometry_msgs/Pose grasp_pose
geometry_msgs/Pose pre_grasp_pose
float32 width
float32 score               # combined antipodal + manipulability + IK + reachability score, see §11
string parent_instance_id
```

### 7.5 `EstimatePose.srv`
```
string label
---
ObjectPose[] poses
bool success
string message
```

### 7.6 `PlanGrasp.srv`
```
string instance_id
uint8 max_attempts
---
GraspCandidate executed
bool success
string message
```

## 8. Online data flow (per pick cycle)

1. BT/GUI calls `EstimatePose.srv(label="BusBar")`.
2. `pose_estimator_node` reads latest `DetectionBundle` filtered to label.
3. Per detection: crop RGB+depth to padded bbox, run FoundationPose `register()`, score, transform to world frame, return `ObjectPose[]`.
4. Caller selects best instance (e.g., score × reachability).
5. BT/GUI calls `PlanGrasp.srv(instance_id, max_attempts=5)`.
6. `grasp_planner_node` loads `grasps.npy` for class, transforms via T_world_obj, filters (collision via CoACD vs scene, IK check, gripper-vs-table clearance), ranks (force closure × manip × distance to current TCP), and loops top-K:
   - `PlanToPose(pre_grasp)`
   - `PlanToPose(grasp, use_cartesian=true)`
   - `CloseGripper.srv`
   - `PlanToPose(retreat, use_cartesian=true)` with `mesh_to_attach=grasps/.../mesh.obj`
7. Returns executed candidate or failure.

## 9. Offline pipeline detail

### 9.1 `decompose_mesh.py`
Calls CoACD with `threshold=0.05, preprocess_resolution=50`. Outputs `collision_decomp.obj`. Goal: ≤32 hulls, total volume within 5% of original (asserted in test).

### 9.2 `generate_grasps.py`
Inputs: `mesh.obj`, `gripper_spec.yaml`.
Algorithm:
1. Surface sample N=5000 points (with normals).
2. For each antipodal pair (`||n_i + n_j|| < ε`):
   - `width = ||p_i - p_j||`; skip if outside gripper limits.
   - Center, approach direction from normals.
   - Sweep 12 wrist roll angles around approach axis → 12 grasp candidates per pair.
3. Collision-check gripper geometry against `collision_decomp.obj` → drop colliding grasps.
4. Compute force-closure score with friction cone (μ from metadata).
5. *Optional* workspace prefilter: for a 10 cm grid of plausible object positions on the table, IK-check each grasp; keep grasps reachable in ≥30% of positions.
6. Output: `grasps.npy` structured array `[{pose_4x4: float32[16], width: float32, score: float32, fc_score: float32}]`. Typical 200–500 grasps per object.

### 9.3 `render_pose_templates.py`
Uses pyrender to render mesh from 42 icosahedral viewpoints, outputting depth + normal templates as `.npz` for FoundationPose initialization.

## 10. FoundationPose integration

- Repo: `https://github.com/NVlabs/FoundationPose` as submodule.
- Deps: pytorch3d, nvdiffrast, kaolin (conda env, isolated from system ROS).
- Per-detection inference (~300–500 ms on 4070):
  ```python
  rgb_crop, depth_crop, K_crop = crop_and_pad(rgb_full, depth_full, K, bbox, pad=0.2)
  pose_cam_obj = fp.register(rgb_crop, depth_crop, K_crop, mesh)
  score = fp.score(rgb_crop, depth_crop, K_crop, mesh, pose_cam_obj)
  ```
- Crop bbox keeps input ≤160×160 → ~5 GB VRAM headroom.
- For tracking subsequent frames: `fp.track(prev_pose)` cheaper than register.
- **Fallback**: if `score < 0.3`, run ICP between cropped depth point cloud and mesh, initialized at YOLO centroid + canonical orientation.

## 11. Grasp planning logic

```
inputs: ObjectPose op, GraspDB grasps[N]
1. grasps_world = [T_world_obj @ g.pose for g in grasps]
2. for g in grasps_world:
     - finger_box collides with scene_octomap? drop
     - finger_box collides with table plane? drop
     - IK exists (UR16e + gantry)? drop if none
3. score(g) = w1 * g.fc_score
            + w2 * (1 - dist(g.pose, current_TCP) / d_max)
            + w3 * manipulability(IK_solution)
            + w4 * vertical_alignment(approach_dir)        # prefer top-down on flat parts
4. return top-K sorted desc
```

Execution loop tries top-K with fallback chain: full plan → cartesian-only retreat → re-look from new viewpoint → fail.

## 12. Performance budget (per pick, 4070 8 GB)

| Stage | Time |
|---|---|
| YOLO detect (existing) | 50 ms |
| Crop + FoundationPose register per object | 300–500 ms |
| Grasp transform + filter (200 → top-20) | 30 ms |
| IK check top-K | 100 ms |
| MoveIt plan pre-grasp | 200 ms |
| MoveIt plan grasp (cartesian) | 100 ms |
| **Total decision** | **~1 s per pick** |

## 13. VRAM budget

| Resident model | VRAM |
|---|---|
| YOLO_WORLD FP16 (existing) | ~2 GB |
| FoundationPose register @ 160² crop | ~5 GB |
| **Total** | **~7 GB (1 GB headroom)** |

Mitigation if OOM: run `pose_estimator_node` in separate process so YOLO can be unloaded when in pose phase; or fall back to SAM6D (option B above).

## 14. Error handling

- **VisionNode**: stale-frame drop + TF lookup retry (already present, extend pattern).
- **PoseEstimator**: low score → don't publish, log warn; 3 consecutive fails → trigger `re_look` action (move arm to alternate viewpoint, retry).
- **GraspPlanner**: all top-K fail planning → request EstimatePose from new viewpoint, retry once, then return failure with reason string.
- BT layer maps failures to retry / abort branches via standard MoveIt error codes.

## 15. Testing strategy

### 15.1 Unit
- `test_grasp_generator.py`: load cube + cylinder meshes; verify N grasps generated, all antipodal, all collision-free.
- `test_pose_filter.py`: mock ObjectPose + mock grasps; verify ranking + filtering deterministic.
- `test_mesh_decomposition.py`: CoACD on busbar mesh → ≤16 hulls, total volume within 5%.
- `test_msg_roundtrip.py`: serialize/deserialize all new msgs.

### 15.2 Integration (Isaac Sim)
- Target simulator: NVIDIA Isaac Sim (replaces earlier Gazebo plan; user has Isaac Sim configured on the deployment laptop).
- Spawn busbar at known ground-truth pose; run full pipeline; assert pose error <5 mm / 5° at 30 cm range.
- Pick-success rate over 50 randomized busbar poses ≥ 90 %.
- Plan 01 (foundation msgs/srvs) does **not** require any sim — runtime verification of `VisionNode` publish and `motion_control_node` mesh-attach behaviour is deferred to the laptop where camera + MoveIt + Isaac Sim are running.

### 15.3 Hardware bring-up
- ArUco fiducial at known location for FoundationPose ground-truth.
- Manual pick attempts on each object class before BT integration.
- **Verify install symlinks after every package add** (per workspace memory: install symlinks point to non-existent `/root/ros2_ws/build/` if not rebuilt correctly).

## 16. Gripper integration (placeholder)

Awaiting final gripper spec. Required info:
- Communication protocol (Modbus TCP / Ethernet / digital IO)
- Position + force feedback availability
- Max grasp force, max stroke, finger geometry
- Opening/closing time
- ros2_control hardware interface availability or need-to-write

URDF/SRDF/MoveIt updates and `OpenGripper.srv`/`CloseGripper.srv` definitions live in a follow-up spec triggered once gripper is finalized. Pipeline above functions with a stub gripper (no-op service) for offline development.

## 17. Risks + open questions

| Risk | Mitigation |
|---|---|
| 4070 8 GB VRAM too tight w/ YOLO + FoundationPose co-resident | Separate process per node; SAM6D fallback (option B) |
| Thin busbars (~2 mm) → CoACD produces degenerate hulls | Inflate by 1 mm before decomp; use mesh directly for FP, decomp only for collision |
| RealSense depth on shiny metal busbars unreliable | Multi-view fusion (move arm, accumulate), optional structured-light projector add-on |
| FoundationPose symmetry ambiguity (busbar 180° flip) | Filter out symmetry-equivalent grasps offline; let grasp planner pick either |
| Workspace prefilter expensive at offline time | Cache per-gripper, regenerate only on URDF change |
| Install symlinks pointing to `/root/ros2_ws/build/` after `colcon build` | Documented in memory; `sudo colcon build --symlink-install --packages-select <pkg>` after every new package |

## 18. Out-of-scope follow-ups

1. Parallel gripper URDF + driver + ros2_control bring-up (separate spec).
2. Place planning (separate spec).
3. Active perception policy (where to move arm for best next view).
4. Multi-object stacking / clutter handling beyond single-pick.
5. VLM-based reasoning for novel classes not in YOLO vocabulary (future research thread).
