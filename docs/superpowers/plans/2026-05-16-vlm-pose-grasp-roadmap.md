# VLM Pose+Grasp Pipeline — Plan Roadmap

Source spec: `docs/superpowers/specs/2026-05-16-vlm-6dpose-grasp-design.md`.

The spec spans 4 independently-testable subsystems. To avoid stale planning (later plans depend on what we learn during earlier ones), full bite-sized plans are written **just-in-time** as each phase begins. This file is the contract for what each plan covers.

| Plan | Status | Outcome | Major task headlines |
|---|---|---|---|
| [01: Msg/srv foundation](2026-05-16-vlm-pose-grasp-plan-01-foundation.md) | Written | All new msgs/srvs land; `VisionNode` publishes `DetectionBundle`; `PlanToPose.srv` extended | T1: `Detection.msg`, T2: `DetectionBundle.msg`, T3: `ObjectPose.msg`, T4: `GraspCandidate.msg`, T5: `EstimatePose.srv`, T6: `PlanGrasp.srv`, T7: install-symlink verify, T8: VisionNode publish fix, T9: PlanToPose.srv extend, T10: motion_control_node attach-mesh handling, T11: integration check |
| **02: Offline CAD tooling** | TODO | CLI tools that take a CAD mesh + gripper spec and emit `collision_decomp.obj`, `grasps.npy`, `pose_templates/`. Standalone Python, no ROS runtime. | (a) Conda env + deps (CoACD, pyrender, kaolin), (b) `tools/decompose_mesh.py` w/ unit tests on cube + cylinder (≤32 hulls, vol within 5%), (c) `gripper_spec.yaml` schema, (d) `tools/generate_grasps.py` antipodal sampler + force-closure score, (e) workspace-IK prefilter (optional), (f) `tools/render_pose_templates.py` (42 icosahedral views), (g) model registry layout under `arpa_grasp_planning/models/<class>/`, (h) integration test on actual busbar CAD → ≥100 grasps, ≥1 reachable in nominal table pose |
| **03: Pose estimator node** | TODO | `arpa_pose_estimation` package with FoundationPose wrapper, `pose_estimator_node`, `EstimatePose.srv` server, `/object_poses` topic publisher. | (a) Submodule + setup.sh for FoundationPose + weights, (b) `foundationpose_wrapper.py` (register + score + track), (c) `model_registry.py` (load mesh + templates + metadata from `models/<class>/`), (d) `pose_estimator_node.py` (sub DetectionBundle → crop+register → pub ObjectPose), (e) EstimatePose.srv server (on-demand mode), (f) instance_id assignment (centroid hash, see spec §7.3), (g) score-threshold fallback to ICP, (h) Gazebo integration test: busbar at known pose → pose error <5 mm / 5°, (i) ArUco fiducial hardware bring-up test |
| **04: Grasp planner + BT integration** | TODO | `arpa_grasp_planning/grasp_planner_node`, `grasp_filter.py`, BT `Pick` action node, end-to-end pick. | (a) `grasp_filter.py` (IK + collision + manip scoring), (b) `grasp_planner_node.py` PlanGrasp.srv server, (c) world-transform of grasp DB given ObjectPose, (d) top-K execution loop w/ pre-grasp → grasp → close → retreat-with-attached-mesh, (e) stub OpenGripper/CloseGripper services (real gripper in separate spec), (f) BT `Pick` action node calling EstimatePose + PlanGrasp, (g) Gazebo end-to-end pick test ≥90% success over 50 random busbar poses, (h) re-look retry policy on planner failure |

## Decomposition rationale

- **Each plan is independently testable** → can sign off on plan 1 before plan 2 starts.
- **Plan 1 unblocks 2 & 3** (msg defs); plan 2 unblocks 3 (grasp DB); plan 3 unblocks 4 (ObjectPose stream).
- **No upfront full plan for 2/3/4** — bite-sized code-level steps for 3 (FoundationPose API specifics) and 4 (gripper-specific) will be wrong if written today before we touch the upstream FoundationPose repo and finalize the gripper.
- **Gripper hardware spec** (driver, ros2_control, finger geometry) is a **separate spec** — Plan 04 uses stub gripper services that the real driver can drop in later.

## Update protocol

When starting Plan 02: brainstorm-skill-style review of any spec changes needed, then this skill (writing-plans) again to fill in the full Plan 02 doc next to this file. Same for 03, 04.
