#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from arpa_control.srv import PlanToPose, ExecutePlan, GetPoseCostMatrix
from custom_ros_messages.srv import EthernetMotor, UR16BehaviorTrigger, RemovePart
from std_srvs.srv import Trigger
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from custom_ros_messages.msg import DetectionBundle
from sensor_msgs.msg import CameraInfo
from moveit_msgs.action import ExecuteTrajectory
try:
    from custom_ros_messages.action import ScanBattery
except ImportError:
    ScanBattery = None
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation
try:
    import open3d as o3d
except ImportError:
    o3d = None

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import json
import threading
import time
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from scipy.spatial.transform import Rotation
import cv2

from std_msgs.msg import String

BASE_FRAME = "floor_link"
EE_FRAME = "tool0"

REMOVE_WAIT_SECONDS = 4
MOTOR_SPEED = 100



class CoreNode(Node):
    def __init__(self):
        super().__init__('core_functionality_node')

        # ReentrantCallbackGroup allows _remove_part_cb to block while client response
        # callbacks (plan, exec, motor, behavior) run concurrently in the same group.
        # Without this, the default MutuallyExclusiveCallbackGroup would deadlock.
        self._reentrant_cb_group = ReentrantCallbackGroup()

        self.plan_client = self.create_client(PlanToPose, 'plan_to_pose', callback_group=self._reentrant_cb_group)
        self.exec_client = self.create_client(ExecutePlan, 'execute_plan', callback_group=self._reentrant_cb_group)
        self.motor_client = self.create_client(EthernetMotor, 'motor_control', callback_group=self._reentrant_cb_group)
        self.behavior_client = self.create_client(UR16BehaviorTrigger, 'ur16e_rest/BehaviorTrigger', callback_group=self._reentrant_cb_group)
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory', callback_group=self._reentrant_cb_group)
        self.update_depth_client = self.create_client(Trigger, 'update_depth', callback_group=self._reentrant_cb_group)
        self.pose_cost_matrix_client = self.create_client(GetPoseCostMatrix, 'get_pose_cost_matrix', callback_group=self._reentrant_cb_group)
        self.planning_scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.create_service(RemovePart, 'remove_part', self._remove_part_cb, callback_group=self._reentrant_cb_group)
        self.remove_part_client = self.create_client(RemovePart, 'remove_part', callback_group=self._reentrant_cb_group)
        self.create_service(Trigger, '/go_home', self._go_home_cb, callback_group=self._reentrant_cb_group)
        self.behavior_publisher = self.create_publisher(String, '/triggered_behavior', 10)
        self.capture_client = self.create_client(Trigger, 'record_images/capture')

        # Declared parameters. Defaults work on the real robot; sim launches can
        # override (e.g. set use_collision_plane:=false if the battery plane
        # interferes with sim planning).
        self.declare_parameter('toolhead_frame', 'test_ratchet_extension_link')
        self.declare_parameter('camera_optical_frame', 'ee_cam_color_optical_frame')
        self.declare_parameter('use_collision_plane', True)
        self.declare_parameter('hover_offset_m', 0.06)
        self.declare_parameter('engage_offset_m', 0.02)
        self.toolhead_frame = self.get_parameter('toolhead_frame').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.use_collision_plane = self.get_parameter('use_collision_plane').value
        self.hover_offset_m = self.get_parameter('hover_offset_m').value
        self.engage_offset_m = self.get_parameter('engage_offset_m').value

        self.save_imgs = False
        self.already_removing = False
        self.already_removing_part_name = ""

 
        self.latest_detection_bundle: DetectionBundle = None
        self.latest_camera_info: CameraInfo = None
        _det_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(DetectionBundle, '/realsense/ee_cam/detections', self._detection_bundle_cb, _det_qos)
        self.create_subscription(CameraInfo, '/realsense/ee_cam/color/camera_info', self._camera_info_cb, _det_qos)

        # self.remove_part_service = self.create_service(
        #     RemovePart, 'remove_part', self._handle_remove_part
        # )

        # Required services (benchmark should fail fast if these aren't up)
        self.get_logger().info("Waiting for plan_to_pose service...")
        if not self.plan_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("Timed out waiting for plan_to_pose service")
        self.get_logger().info("Waiting for execute_plan service...")
        if not self.exec_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("Timed out waiting for execute_plan service")

        # Optional services (available on real robot / full stack; skip in sim if missing)
        self.get_logger().info("Checking optional services...")
        self._motor_available = self.motor_client.wait_for_service(timeout_sec=2.0)
        if not self._motor_available:
            self.get_logger().warn("motor_control service not available (continuing without motor control)")

        self._behavior_available = self.behavior_client.wait_for_service(timeout_sec=2.0)
        if not self._behavior_available:
            self.get_logger().warn("ur16e_rest/BehaviorTrigger service not available (continuing without behavior triggers)")

        self._update_depth_available = self.update_depth_client.wait_for_service(timeout_sec=2.0)
        if not self._update_depth_available:
            self.get_logger().warn("update_depth service not available (continuing without depth updates)")

        self._pose_cost_matrix_available = self.pose_cost_matrix_client.wait_for_service(timeout_sec=5.0)
        if not self._pose_cost_matrix_available:
            self.get_logger().warn("get_pose_cost_matrix service not available (will use pose list order without TSP optimization)")

        # Servo node
        # self.servo_twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        # self.servo_status_sub = self.create_subscription(Int8, '/servo_node/status', self._servo_status_cb, 10)
        # self._servo_status: int = -1
        # self._servo_started: bool = False
        # self.servo_start_client = self.create_client(Trigger, '/servo_node/start_servo', callback_group=self._reentrant_cb_group)
        # self.servo_stop_client = self.create_client(Trigger, '/servo_node/stop_servo', callback_group=self._reentrant_cb_group)


        self.get_logger().info("Core services ready!")


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # MultiThreadedExecutor so service callbacks (e.g. _remove_part_cb) can block on
        # nested service calls without starving the executor of threads to process responses.
        self._executor = rclpy.executors.MultiThreadedExecutor(5)
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self.get_logger().info(f"Looking up wrist_3_link -> {self.toolhead_frame} transform...")
        self.T_wrist3_to_toolhead = None
        self.T_wrist3_to_camera_optical = None
        while self.T_wrist3_to_toolhead is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.toolhead_frame, 'wrist_3_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                t = tf.transform.translation
                r = tf.transform.rotation
                rot = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
                mat = np.eye(4)
                mat[:3, :3] = rot
                mat[:3,  3] = [t.x, t.y, t.z]
                self.T_wrist3_to_toolhead = mat
                self.get_logger().info(f"wrist_3_link -> {self.toolhead_frame}:\n{mat}")
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"TF not ready yet: {e}. Retrying...")
                time.sleep(0.5)
        self.T_toolhead_to_wrist3 = np.linalg.inv(self.T_wrist3_to_toolhead)

        # Camera optical frame is optional: present on the real robot (EE RealSense)
        # but absent in sim. Try briefly, then continue without it on miss.
        self.get_logger().info(
            f"Looking up wrist_3_link -> {self.camera_optical_frame} transform (optional)..."
        )
        camera_lookup_deadline = time.time() + 5.0
        while self.T_wrist3_to_camera_optical is None and time.time() < camera_lookup_deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.camera_optical_frame, 'wrist_3_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                t = tf.transform.translation
                r = tf.transform.rotation
                rot = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
                mat = np.eye(4)
                mat[:3, :3] = rot
                mat[:3,  3] = [t.x, t.y, t.z]
                self.T_wrist3_to_camera_optical = mat
                self.get_logger().info(f"wrist_3_link -> {self.camera_optical_frame}:\n{mat}")
            except (LookupException, ConnectivityException, ExtrapolationException):
                time.sleep(0.5)

        if self.T_wrist3_to_camera_optical is not None:
            self.T_camera_optical_to_wrist3 = np.linalg.inv(self.T_wrist3_to_camera_optical)
        else:
            self.T_camera_optical_to_wrist3 = None
            self.get_logger().warn(
                f"Camera frame '{self.camera_optical_frame}' unavailable — "
                f"visual servo will be skipped."
            )

        # Visualization / servo state
        self.obj_bbox_point = None
        self.target_pixel = None
        self.ref_pixel = None
        self.PIXEL_TOLERANCE = 128.0  # pixels
        self.PIXEL_CONVERGENCE = 8.0

        # target_pixel is the projection of the toolhead origin into the camera image,
        # used as the alignment goal for IBVS. Only computable when both camera TF
        # and intrinsics are available.
        if self.T_wrist3_to_camera_optical is not None:
            self.get_logger().info("Waiting briefly for camera info...")
            camera_info_deadline = time.time() + 5.0
            while self.latest_camera_info is None and time.time() < camera_info_deadline:
                time.sleep(0.1)
            if self.latest_camera_info is not None:
                T_toolhead_in_cam = self.T_wrist3_to_camera_optical @ self.T_toolhead_to_wrist3
                tx, ty, tz = T_toolhead_in_cam[:3, 3]
                ifx, ify = self.latest_camera_info.k[0], self.latest_camera_info.k[4]
                icx, icy = self.latest_camera_info.k[2], self.latest_camera_info.k[5]
                TARGET_PIXEL_OFFSET = np.array([-30.0, -25.0])  # [left, up] in pixels
                self.target_pixel = np.array([ifx * tx / tz + icx,
                                              ify * ty / tz + icy]) + TARGET_PIXEL_OFFSET
                self.get_logger().info(
                    f"target_pixel set to toolhead projection: {self.target_pixel}"
                )
            else:
                self.get_logger().warn(
                    "Camera info not received within timeout — target_pixel left as None."
                )
        else:
            self.get_logger().info("Camera frame unavailable — target_pixel left as None.")

    def _detection_bundle_cb(self, msg: DetectionBundle):
        self.latest_detection_bundle = msg

    def _camera_info_cb(self, msg: CameraInfo):
        self.latest_camera_info = msg

    # def _servo_status_cb(self, msg: Int8):
    #     self._servo_status = msg.data

    def trigger_with_retry(self, behavior, retries=3):
        # Early bail when no rest controller is in the system (sim) — avoids
        # 3× (timeout + sleep) wasted log spam per behavior call.
        if not getattr(self, "_behavior_available", False):
            self.get_logger().debug(
                f"Behavior '{behavior}' skipped (no BehaviorTrigger service)"
            )
            return False
        for attempt in range(1, retries + 1):
            if self.trigger_behavior(behavior):
                return True
            self.get_logger().warn(f"Behavior '{behavior}' failed (attempt {attempt}/{retries}), retrying...")
            time.sleep(1)
        self.get_logger().error(f"Behavior '{behavior}' failed after {retries} attempts.")
        return False
    
    def make_pose_stamped(self, frame_id, x, y, z, qx, qy, qz, qw):
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
    

    def _plan_and_execute(self, pose_stamped, retries=3):
        """Plan toolhead to pose_stamped (with up to `retries` planning attempts),
        then execute the plan. Returns True on full success, False otherwise."""
        p = pose_stamped.pose.position
        q = pose_stamped.pose.orientation
        frame_id = pose_stamped.header.frame_id or BASE_FRAME
        for attempt in range(1, retries + 1):
            if self.plan_toolhead_to_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w, frame_id=frame_id):
                return self.execute_plan()
            self.get_logger().warn(
                f"Planning to ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) failed (attempt {attempt}/{retries})"
            )
        return False


    def _remove_part_cb(self, request: RemovePart.Request, response: RemovePart.Response):
        if self.already_removing:
            self.get_logger().warn(
                f"System is already removing part {self.already_removing_part_name}. "
                f"Not processing new request."
            )
            response.success = False
            response.message = "Currently in progress removing part = " + self.already_removing_part_name
            return response

        self.already_removing = True
        self.already_removing_part_name = request.part_name
        self.get_logger().info(
            f"Received request to remove part '{request.part_name}' "
            f"with confidence {request.detection_confidence:.2f}."
        )
        # Signal new trial (no-op when no /triggered_behavior subscribers)
        self.behavior_publisher.publish(String(
            data=f"Remove: {request.part_name} with confidence = {request.detection_confidence}"
        ))

        try:
            pose = request.target_pose
            x = pose.pose.position.x
            y = pose.pose.position.y
            screw_z = pose.pose.position.z
            hover_z = screw_z + self.hover_offset_m
            at_z = screw_z + self.engage_offset_m
            frame_id = pose.header.frame_id or BASE_FRAME

            # Orientation: if the caller left it at identity (w=1.0), synthesize a
            # tool-down quaternion pointing toward the gantry origin; otherwise
            # honor the caller's orientation.
            if pose.pose.orientation.w == 1.0:
                z_hat = np.array([0.0, 0.0, -1.0])
                toward_origin = np.array([-x, -y, 0.0])
                norm = np.linalg.norm(toward_origin)
                y_hat = toward_origin / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0])
                x_hat = np.cross(y_hat, z_hat)
                R = np.column_stack([x_hat, y_hat, z_hat])
                qx, qy, qz, qw = Rotation.from_matrix(R).as_quat()
            else:
                qx = pose.pose.orientation.x
                qy = pose.pose.orientation.y
                qz = pose.pose.orientation.z
                qw = pose.pose.orientation.w

            hover_pose = self.make_pose_stamped(frame_id, x, y, hover_z, qx, qy, qz, qw)
            engage_pose = self.make_pose_stamped(frame_id, x, y, at_z, qx, qy, qz, qw)

            # ── Stage 1: plan + execute to hover above the screw ──
            if not self._plan_and_execute(hover_pose, retries=3):
                response.success = False
                response.message = "Planning to hover failed after 3 attempts"
                return response
            time.sleep(0.5)

            # ── Stage 2: pre-servo image capture (no-op without subscriber) ──
            self.behavior_publisher.publish(String(data="capture_images:pre_servo"))
            time.sleep(0.5)

            # ── Stage 3: optional visual servo (needs EE camera + intrinsics) ──
            servo_capable = (
                request.visual_servo
                and self.T_wrist3_to_camera_optical is not None
                and self.target_pixel is not None
            )
            if servo_capable:
                self.behavior_publisher.publish(String(data="start_servo_loop"))
                time.sleep(0.25)
                screw_pose = self.make_pose_stamped(frame_id, x, y, screw_z, qx, qy, qz, qw)
                screw_pose.header = pose.header
                alignment_result = self.align_to_screw_img(initial_screw_pose=screw_pose)
                if alignment_result:
                    self.behavior_publisher.publish(String(data="abort_servo_loop"))
                else:
                    self.behavior_publisher.publish(String(data="stop_servo_loop"))
                time.sleep(0.25)
                self.get_logger().info(
                    f"Visual servo done, alignment_result={alignment_result}. "
                    f"Proceeding with removal (screw may be occluded when aligned)."
                )
            elif request.visual_servo:
                self.get_logger().warn(
                    "Visual servo requested but EE camera unavailable — skipping."
                )

            # ── Stage 4: post-servo image capture ──
            self.behavior_publisher.publish(String(data="capture_images:post_servo"))
            time.sleep(0.5)

            # ── Stage 5: start recording torque / robot state ──
            self.behavior_publisher.publish(String(data="start_recording"))
            time.sleep(0.25)

            # ── Stage 6: engage. On real, the rest tool descends under force
            # control via ZForce/play; without rest behaviors (sim), we descend
            # cartesianly to at_z so motion is still observable.
            self.motor_control(MOTOR_SPEED)  # no-op if motor service unavailable
            if self._behavior_available:
                self.trigger_with_retry("ZForce")
                self.trigger_with_retry("play")
                time.sleep(REMOVE_WAIT_SECONDS)
            else:
                self.get_logger().info("No rest behaviors — cartesian descent to engage")
                if not self._plan_and_execute(engage_pose, retries=3):
                    self.get_logger().warn("Engage descent failed; continuing to retract")
                time.sleep(REMOVE_WAIT_SECONDS)

            # ── Stage 7: pre-retract image capture ──
            self.behavior_publisher.publish(String(data="capture_images:pre_retract"))
            time.sleep(0.5)

            # ── Stage 8: retract. On real: retract/play via rest tool. Sim:
            # cartesian retreat back to hover.
            if self._behavior_available:
                self.trigger_with_retry("retract")
                self.trigger_with_retry("play")
                time.sleep(REMOVE_WAIT_SECONDS)
            else:
                self.get_logger().info("No rest behaviors — cartesian retreat to hover")
                if not self._plan_and_execute(hover_pose, retries=3):
                    self.get_logger().warn("Retreat to hover failed")

            self.motor_control(0)  # no-op if motor service unavailable
            if self._behavior_available:
                self.trigger_with_retry("ros2control")

            # ── Stage 9: stop recording, announce completion ──
            self.behavior_publisher.publish(String(data="stop_recording"))
            self.behavior_publisher.publish(String(data=f"Removed: {request.part_name}"))

            response.success = True
            response.message = "Remove Part complete"

        except Exception as e:
            self.get_logger().error(f"FAILED TO REMOVE PART: {e}")
            response.success = False
            response.message = f"Remove part failed: {e}"
        finally:
            self.already_removing = False
            self.already_removing_part_name = ""

        return response

    # def _start_servo(self) -> bool:
    #     """Call start_servo service and wait for it. Returns True on success."""
    #     if not self.servo_start_client.wait_for_service(timeout_sec=2.0):
    #         self.get_logger().error("start_servo service not available")
    #         return False
    #     future = self.servo_start_client.call_async(Trigger.Request())
    #     while not future.done():
    #         time.sleep(0.05)
    #     self.get_logger().info(f"start_servo: {future.result().message}")
    #     self._servo_started = True
    #     return True

    # def servo_twist(self, x: float, y: float, z: float,
    #                 roll: float, pitch: float, yaw: float,
    #                 frame_id: str = EE_FRAME):
    #     # Re-start servo if not started or if it has halted (status 2=singularity, 5=collision, 6=joint bound)
    #     servo_halted = self._servo_status in (2, 5, 6)
    #     if not self._servo_started or servo_halted:
    #         if servo_halted:
    #             self.get_logger().warn(f"Servo halted (status={self._servo_status}), restarting...")
    #         else:
    #             self.get_logger().info("Starting servo...")
    #         if not self._start_servo():
    #             return
    #         time.sleep(0.05)  # brief settle before publishing

    #     all_zero = (x == 0.0 and y == 0.0 and z == 0.0 and
    #                 roll == 0.0 and pitch == 0.0 and yaw == 0.0)

    #     if all_zero:
    #         if not self.servo_stop_client.wait_for_service(timeout_sec=2.0):
    #             self.get_logger().warn("stop_servo service not available")
    #             return
    #         future = self.servo_stop_client.call_async(Trigger.Request())
    #         while not future.done():
    #             time.sleep(0.05)
    #         self.get_logger().info(f"stop_servo: {future.result().message}")
    #         self._servo_started = False  # force re-start on next use
    #         return

    #     msg = TwistStamped()
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.header.frame_id = frame_id
    #     msg.twist.linear.x = x
    #     msg.twist.linear.y = y
    #     msg.twist.linear.z = z
    #     msg.twist.angular.x = roll
    #     msg.twist.angular.y = pitch
    #     msg.twist.angular.z = yaw
    #     self.servo_twist_pub.publish(msg)

    def plan_to_pose(self, x, y, z, qx, qy, qz, qw, frame_id="world"):
        req = PlanToPose.Request()
        req.target_pose.header.frame_id = frame_id
        req.target_pose.pose.position.x = x
        req.target_pose.pose.position.y = y
        req.target_pose.pose.position.z = z
        req.target_pose.pose.orientation.x = qx
        req.target_pose.pose.orientation.y = qy
        req.target_pose.pose.orientation.z = qz
        req.target_pose.pose.orientation.w = qw

        self.get_logger().info(f"Planning to pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                               f"qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f} "
                               f"in frame '{frame_id}'")

        future = self.plan_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

        result = future.result()
        if result.success:
            self.get_logger().info("Planning successful!")
        else:
            self.get_logger().error(f"Planning failed: {result.message}")
        return result.success

    def plan_camera_to_pose(self, x, y, z, qx, qy, qz, qw, frame_id="world"):
        # Convert camera pose to wrist_3_link pose using the known transform
        target_camera = np.eye(4)
        target_camera[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        target_camera[:3, 3] = [x, y, z]
        target_wrist3 = target_camera @ self.T_wrist3_to_camera_optical

        wx, wy, wz = target_wrist3[:3, 3]
        rot = target_wrist3[:3, :3]
        qx, qy, qz, qw = Rotation.from_matrix(rot).as_quat()

        return self.plan_to_pose(wx, wy, wz, qx, qy, qz, qw, frame_id)

    def plan_toolhead_to_pose(self, x, y, z, qx, qy, qz, qw, frame_id="world"):
        # Convert toolhead pose to wrist_3_link pose using the known transform
        target_toolhead = np.eye(4)
        target_toolhead[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        target_toolhead[:3, 3] = [x, y, z]
        target_wrist3 = target_toolhead @ self.T_wrist3_to_toolhead


        wx, wy, wz = target_wrist3[:3, 3]
        rot = target_wrist3[:3, :3]
        qx, qy, qz, qw = Rotation.from_matrix(rot).as_quat()

        return self.plan_to_pose(wx, wy, wz, qx, qy, qz, qw, frame_id)

    def execute_plan(self):
        req = ExecutePlan.Request()

        self.get_logger().info("Executing plan...")
        future = self.exec_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

        result = future.result()
        if result.success:
            self.get_logger().info("Execution successful!")
        else:
            self.get_logger().error(f"Execution failed: {result.message}")
        return result.success

    def execute_trajectory(self, trajectory):
        """Execute a RobotTrajectory directly via MoveIt's ExecuteTrajectory action."""
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ExecuteTrajectory action server not available")
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info("Sending trajectory for execution...")
        future = self.execute_trajectory_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory execution goal rejected")
            return False

        self.get_logger().info("Trajectory accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        result = result_future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Trajectory execution successful!")
            return True
        else:
            self.get_logger().error(f"Trajectory execution failed: error_code={result.error_code.val}")
            return False

    def motor_control(self, speed):
        if not getattr(self, "_motor_available", False):
            self.get_logger().warn("Motor control requested but motor_control service is unavailable (skipping)")
            return False
        if not self.motor_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Motor control service not available")
            return False

        req = EthernetMotor.Request()
        req.enable = speed > 0
        req.speed = speed

        action = "ON" if speed > 0 else "OFF"
        self.get_logger().info(f"Motor {action} at speed {speed}")

        future = self.motor_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

        result = future.result()
        if result.success:
            self.get_logger().info(f"Motor: {result.message}")
        else:
            self.get_logger().error(f"Motor failed: {result.message}")
        return result.success

    def update_depth(self):
        if not getattr(self, "_update_depth_available", False):
            self.get_logger().warn("Depth update requested but update_depth service is unavailable (skipping)")
            return False
        if not self.update_depth_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Update depth service not available")
            return False

        req = Trigger.Request()
        self.get_logger().info("Updating depth map...")

        future = self.update_depth_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

        result = future.result()
        if result.success:
            self.get_logger().info(f"Depth update: {result.message}")
        else:
            self.get_logger().error(f"Depth update failed: {result.message}")
        return result.success

    def add_collision_plane(self, plane_id = None, frame_id = None, x=None, y=None, z=None, size_x=None, size_y=None, thickness=None):
        plane_id = "battery_do_not_cross" if plane_id is None else plane_id
        frame_id = "floor_link" if frame_id is None else frame_id
        x = 0.118 if x is None else x
        y = -0.056 if y is None else y
        z = 0.87 if z is None else z
        size_x = 2.182 if size_x is None else size_x
        size_y = 1.574 if size_y is None else size_y
        thickness = 0.04 if thickness is None else thickness

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = plane_id
        collision_object.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [size_x, size_y, thickness]

        box_pose = Pose()
        box_pose.position.x = x
        box_pose.position.y = y
        box_pose.position.z = z
        box_pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)

        self.planning_scene_pub.publish(planning_scene)
        self.get_logger().info(f"Added collision plane '{plane_id}' at z={z}")

    def remove_collision_plane(self, plane_id, frame_id):
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = plane_id
        collision_object.operation = CollisionObject.REMOVE

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)

        self.planning_scene_pub.publish(planning_scene)
        self.get_logger().info(f"Removed collision plane '{plane_id}'")

    def go_home(self, frame_kwrd):
        self.get_logger().info("Going home...")
        plan_success = False
        exec_success = False
        if frame_kwrd == "wrist_3_link":
            plan_success = self.plan_to_pose(1.112, -0.573, 1.253, 0.7071068, 0.7071068, 0.0, 0.0, frame_id="floor_link")
            exec_success = self.execute_plan()
        elif frame_kwrd == "test_ratchet_extension_link":
            plan_success = self.plan_toolhead_to_pose(1.112, -0.573, 1.253, 0.7071068, 0.7071068, 0.0, 0.0, frame_id="floor_link")
            exec_success = self.execute_plan()
        elif frame_kwrd == "ee_cam_color_optical_frame":
            #TODO use so that all home poses have same orientation 
            #rot_90_z = Rotation.from_euler('z', -90, degrees=True).as_matrix()

            plan_success = self.plan_camera_to_pose(1.112, -0.573, 1.253, 0.7071068, 0.7071068, 0.0, 0.0, frame_id="floor_link")
            exec_success = self.execute_plan()
        else:
            print(f"Unknown frame keyword '{frame_kwrd}' for go_home")
        self.get_logger().error(f"go home {plan_success=}, {exec_success=}")
        return plan_success and exec_success

    def _go_home_cb(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().warn("/go_home Trigger uses default frame 'wrist_3_link'")
        success = self.go_home("wrist_3_link")
        response.success = success
        response.message = "Go home complete" if success else "Go home failed"
        return response

    def trigger_behavior(self, behavior):
        if not getattr(self, "_behavior_available", False):
            self.get_logger().warn(f"Behavior '{behavior}' requested but BehaviorTrigger service is unavailable (skipping)")
            return False
        if not self.behavior_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Behavior service not available")
            return False

        req = UR16BehaviorTrigger.Request()
        req.behavior = behavior

        self.get_logger().info(f"Triggering behavior: {behavior}")

        future = self.behavior_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

        result = future.result()
        if result.success:
            self.get_logger().info(f"Behavior: {result.message}")
        else:
            self.get_logger().error(f"Behavior failed: {result.message}")


        #TODO play behaviors without the need to call play independently
        # if not behavior in  ("ros2control", "externalcontrol", "play"):
        #     return result.success
        
        return result.success
    

    def align_to_screw_img(self, initial_screw_pose=None):
        # Lock orientation from the first bundle so it doesn't drift across steps
        bundle = self.latest_detection_bundle
        if bundle is None:
            self.get_logger().error("No detection bundle available")
            return False
        cp = bundle.camera_pose.pose
        locked_quat = (cp.orientation.x, cp.orientation.y, cp.orientation.z, cp.orientation.w)

        alignment_in_progress = True
        max_steps = 100
        current_step = 0
        while alignment_in_progress and max_steps > current_step:
            alignment_in_progress = self._one_step_align_to_screw_img(initial_screw_pose, locked_quat)
            current_step += 1
            time.sleep(0.25)
        return alignment_in_progress

    def _one_step_align_to_screw_img(self, initial_screw_pose=None, locked_quat=None):
        STEP_GAIN = 0.3
        MAX_STEP = 0.05

        bundle = self.latest_detection_bundle
        if bundle is None:
            self.get_logger().error("No detection bundle available")
            return False

        detections = [d for d in bundle.detections if d.cls.lower() in ["screw", "nut"]]
        if not detections:
            self.get_logger().warn("No detections in bundle")
            return False


        # --- Camera intrinsics and pose ---
        cp = bundle.camera_pose.pose
        R_cam_world = Rotation.from_quat([cp.orientation.x, cp.orientation.y,
                                      cp.orientation.z, cp.orientation.w]).as_matrix()
        T_cam_to_world = np.eye(4)
        T_cam_to_world[:3, :3] = R_cam_world
        T_cam_to_world[:3, 3]  = [cp.position.x, cp.position.y, cp.position.z]
        ifx, ify = bundle.camera_info.k[0], bundle.camera_info.k[4]
        icx, icy = bundle.camera_info.k[2], bundle.camera_info.k[5]

        T_world_to_cam = np.linalg.inv(T_cam_to_world)

        # Use locked orientation for world_delta so it is consistent with plan_camera_to_pose
        R_for_delta = Rotation.from_quat(locked_quat).as_matrix() if locked_quat is not None else R_cam_world
        
        
        
        # --- Reference pixel for bbox selection ---
        if initial_screw_pose is not None:
            p = initial_screw_pose.pose.position
            p_cam = T_world_to_cam @ np.array([p.x, p.y, p.z, 1.0])
            if p_cam[2] <= 0:
                self.get_logger().error(f"Screw is behind the camera (p_cam_Z={p_cam[2]:.3f})")
                return False
            self.ref_pixel = np.array([ifx * p_cam[0] / p_cam[2] + icx,
                                  ify * p_cam[1] / p_cam[2] + icy])
            self.get_logger().info(
                f"ref_pixel=({self.ref_pixel[0]:.1f}, {self.ref_pixel[1]:.1f})  "
                f"screw_world=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
            )
        else:
            self.ref_pixel = self.target_pixel
            self.get_logger().info(f"ref_pixel=target_pixel=({self.ref_pixel[0]:.1f}, {self.ref_pixel[1]:.1f})")

        self.get_logger().info(
            f"target_pixel=({self.target_pixel[0]:.1f}, {self.target_pixel[1]:.1f})  "
            f"cam_pos=({cp.position.x:.3f}, {cp.position.y:.3f}, {cp.position.z:.3f})"
        )

        # --- Match closest bounding box ---
        best_det, best_dist, best_pixel = None, float('inf'), None
        for det in detections:
            bbox_center = np.array([(det.bbox_min.x + det.bbox_max.x) / 2.0,
                                    (det.bbox_min.y + det.bbox_max.y) / 2.0])
            dist = np.linalg.norm(bbox_center - self.ref_pixel)
            self.get_logger().info(f"  det '{det.cls}' bbox=({bbox_center[0]:.1f}, {bbox_center[1]:.1f}) dist={dist:.1f}px")
            if dist < best_dist:
                best_dist, best_det, best_pixel = dist, det, bbox_center
        if best_det is None or (best_dist > self.PIXEL_TOLERANCE and initial_screw_pose is not None):
            self.get_logger().error(f"No detection within {self.PIXEL_TOLERANCE}px of ref_pixel (closest={best_dist:.1f}px)")
            return False
        obj_bbox_pixel = best_pixel
        dist_to_target = np.linalg.norm(obj_bbox_pixel - self.target_pixel)
        self.get_logger().info(
            f"Matched '{best_det.cls}' ({best_det.prob:.2f})  "
            f"bbox=({obj_bbox_pixel[0]:.1f}, {obj_bbox_pixel[1]:.1f})  "
            f"dist_to_ref={best_dist:.1f}px  dist_to_target={dist_to_target:.1f}px"
        )

        if dist_to_target < self.PIXEL_CONVERGENCE:
            self.get_logger().info(f"Already aligned (dist_to_target={dist_to_target:.1f}px < {self.PIXEL_CONVERGENCE}px), skipping move.")
            return False

        # --- Z from mean of valid depth pixels in bbox ROI ---
        Z = None
        depth_buf = np.frombuffer(bytes(bundle.depth_image.data)[12:], dtype=np.uint8)
        depth_raw = cv2.imdecode(depth_buf, cv2.IMREAD_UNCHANGED)  # uint16 mm
        if depth_raw is not None:
            u0 = int(np.clip(best_det.bbox_min.x, 0, depth_raw.shape[1] - 1))
            v0 = int(np.clip(best_det.bbox_min.y, 0, depth_raw.shape[0] - 1))
            u1 = int(np.clip(best_det.bbox_max.x, 0, depth_raw.shape[1]))
            v1 = int(np.clip(best_det.bbox_max.y, 0, depth_raw.shape[0]))
            roi = depth_raw[v0:v1, u0:u1]
            valid = roi[roi > 0]
            valid = valid[valid < 500]  # exclude background beyond 0.5 m
            if len(valid) > 0:
                Z = float(np.median(valid)) / 1000.0  # mm → m
                self.get_logger().info(f"Depth from image: Z={Z:.3f}m ({len(valid)} valid px)")
            else:
                self.get_logger().warn("No valid depth pixels in bbox ROI")
        else:
            self.get_logger().warn("Failed to decode depth image")

        if Z is None:
            if initial_screw_pose is not None:
                # Geometric fallback: optical-axis depth from camera height above screw
                screw_z_world = initial_screw_pose.pose.position.z
                cam_z_world = cp.position.z
                Z = max(0.05, cam_z_world - screw_z_world)
                self.get_logger().warn(f"Using geometric Z fallback: {Z:.3f}m")
            else:
                Z = 0.2
                self.get_logger().warn("Using hardcoded Z fallback: 0.200m")


        # --- IBVS one-shot control law (ViSP formulation) ---
        #   Normalized image coords:  x = (u - u0) / fx,  y = (v - v0) / fy
        #   Feature error:            e = s - s*   (current - desired)
        #   Interaction matrix:       Lx = [[-1/Z, 0, x/Z], [0, -1/Z, y/Z]]
        #   Control law:              vc = -lambda * pinv(Lx) * e   (lam=1 → one-shot)
        s      = np.array([(obj_bbox_pixel[0] - icx) / ifx,
                           (obj_bbox_pixel[1] - icy) / ify])
        s_star = np.array([(self.target_pixel[0]   - icx) / ifx,
                           (self.target_pixel[1]   - icy) / ify])
        e      = s - s_star

        L  = np.array([[-1/Z,    0,     s_star[0]/Z],
                       [   0, -1/Z, s_star[1]/Z]])
        vc_cam = -np.linalg.pinv(L) @ e
        vc_cam[2] = 0.0
        # Rotate IBVS delta to world frame and apply to current camera world position
        # Use locked orientation matrix so world_delta is consistent with plan_camera_to_pose
        delta_cam = STEP_GAIN * vc_cam
        step_norm = np.linalg.norm(delta_cam)
        if step_norm > MAX_STEP:
            delta_cam *= MAX_STEP / step_norm
        world_delta = R_for_delta @ delta_cam
        target_cam_pose = T_cam_to_world[:3, 3] + world_delta
        cx, cy, _ = target_cam_pose
        self.get_logger().info(
            f"IBVS  Z={Z:.3f}  e=({e[0]:.4f}, {e[1]:.4f})  "
            f"{target_cam_pose=}"
            f"vc_cam=({vc_cam[0]:.4f}, {vc_cam[1]:.4f})  "
            f"→ world ({cx:.4f}, {cy:.4f})"
        )

        # Keep orientation locked from loop start — prevents Z drift via cam→wrist transform
        if locked_quat is not None:
            qx, qy, qz, qw = locked_quat
        else:
            qx, qy, qz, qw = Rotation.from_matrix(T_cam_to_world[:3, :3]).as_quat()

        # Calculate world position of bbox center
        p_cam = np.array([(obj_bbox_pixel[0] - icx) / ifx * Z,
                          (obj_bbox_pixel[1] - icy) / ify * Z,
                          Z,
                          1.0])
        p_world = T_cam_to_world @ p_cam
        self.obj_bbox_point = p_world[:3]

        # Publish servo step data for VLA data collection
        pixel_error = float(dist_to_target)
        servo_payload = json.dumps({
            "pixel_error": pixel_error,
            "depth_Z": float(Z),
            "velocity_cmd": [float(world_delta[0]), float(world_delta[1]), float(world_delta[2])],
        })
        self.behavior_publisher.publish(String(data=f"servo_step:{servo_payload}"))

        cam_z = T_cam_to_world[2, 3]  # maintain current height
        plan_success = self.plan_camera_to_pose(cx, cy, cam_z, qx, qy, qz, qw)
        if not plan_success:
            self.get_logger().error("Failed to plan to above-screw pose")
            return False
        return self.execute_plan()

    def get_tsp_order(self, poses, euclidean=True):
        """
        poses: list of PoseStamped

        returns: list of PoseStamped in optimal visit order from current robot position
        """
        if not getattr(self, "_pose_cost_matrix_available", False):
            return poses

        # Get current EE pose as start node (required for TSP). If TF not ready, use pose order.
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                EE_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF not available for TSP ({e}). Using pose list order."
            )
            return poses

        start_pose = PoseStamped()
        start_pose.header.frame_id = BASE_FRAME
        start_pose.pose.position.x = transform.transform.translation.x
        start_pose.pose.position.y = transform.transform.translation.y
        start_pose.pose.position.z = transform.transform.translation.z
        start_pose.pose.orientation = transform.transform.rotation
        self.get_logger().info(
            f"Current EE position: ({start_pose.pose.position.x:.3f}, "
            f"{start_pose.pose.position.y:.3f}, {start_pose.pose.position.z:.3f})")

        all_poses = [start_pose] + list(poses)
        n = len(all_poses)
        dummy_end_idx = n  # virtual node — no physical location

        # Get full pairwise cost matrix in a single service call
        self.get_logger().info(f"Requesting {n}x{n} cost matrix from service...")
        req = GetPoseCostMatrix.Request()
        req.poses = all_poses
        req.euclidean = euclidean
        start_time = time.time()
        future = self.pose_cost_matrix_client.call_async(req)
        while not future.done():
            time.sleep(0.05)
        end_time = time.time()

        result = future.result()
        self.get_logger().info(f"Cost matrix computed in {end_time - start_time:.2f} seconds")
        if not result.success:
            self.get_logger().error(f"GetPoseCostMatrix failed: {result.message}")
            raise ValueError(f"GetPoseCostMatrix failed: {result.message}")

        self.get_logger().info(f"Cost matrix received: {result.message}")

        # Reshape flat row-major array into 2D cost matrix, add dummy end column/row
        flat = result.cost_matrix
        cost_matrix = [[0.0] * (n + 1) for _ in range(n + 1)]
        for i in range(n):
            for j in range(n):
                cost_matrix[i][j] = flat[i * n + j]

        manager = pywrapcp.RoutingIndexManager(
            n + 1,          # nodes: start + targets + dummy end
            1,              # one vehicle
            [0],            # start depot
            [dummy_end_idx] # end depot (dummy)
        )
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            i = manager.IndexToNode(from_index)
            j = manager.IndexToNode(to_index)
            if i == dummy_end_idx or j == dummy_end_idx:
                return 0
            return int(cost_matrix[i][j] * 1000)

        cb_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(cb_index)

        params = pywrapcp.DefaultRoutingSearchParameters()
        params.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        params.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        params.time_limit.seconds = 5

        solution = routing.SolveWithParameters(params)
        if not solution:
            self.get_logger().error("TSP solver found no solution")
            raise ValueError("TSP SOLVER ERROR")

        # Walk the route, skipping node 0 (start) and dummy end
        ordered = []
        index = solution.Value(routing.NextVar(routing.Start(0)))  # skip start
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node != dummy_end_idx:
                ordered.append(poses[node - 1])  # -1: node 0 is start
            index = solution.Value(routing.NextVar(index))
        return ordered

    def visualize_detections(self):
        def visualize_detections_thread_func():
            bridge = CvBridge()
            while rclpy.ok():
                if self.latest_detection_bundle is not None:
                    rgb_msg = self.latest_detection_bundle.rgb_image
                    depth_msg = self.latest_detection_bundle.depth_image
                    detections = self.latest_detection_bundle.detections
                    rgb_img = bridge.compressed_imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                    try:
                        depth_buf = np.frombuffer(bytes(depth_msg.data)[12:], dtype=np.uint8)
                        depth_raw = cv2.imdecode(depth_buf, cv2.IMREAD_UNCHANGED)  # uint16 mm
                        if depth_raw is None:
                            raise ValueError("imdecode returned None")
                        depth_norm = cv2.normalize(depth_raw, None, 0, 255, cv2.NORM_MINMAX)
                        depth_u8 = depth_norm.astype(np.uint8)
                        depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
                    except Exception:
                        depth_color = np.zeros((rgb_img.shape[0], rgb_img.shape[1], 3), dtype=np.uint8)
                    for det in detections:
                        x1, y1 = int(det.bbox_min.x), int(det.bbox_min.y)
                        x2, y2 = int(det.bbox_max.x), int(det.bbox_max.y)
                        label = f"{det.cls} {det.prob:.2f}"
                        for img in (rgb_img, depth_color):
                            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    for img in (rgb_img, depth_color):
                        if self.target_pixel is not None:
                            tp = (int(self.target_pixel[0]), int(self.target_pixel[1]))
                            cv2.drawMarker(img, tp, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                            cv2.putText(img, "target", (tp[0] + 6, tp[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                        if self.target_pixel is not None:
                            tp = (int(self.target_pixel[0]), int(self.target_pixel[1]))
                            cv2.circle(img, tp, int(self.PIXEL_CONVERGENCE), (0, 255, 255), 1)
                            cv2.circle(img, tp, int(self.PIXEL_TOLERANCE), (0, 165, 255), 1)
                        if self.ref_pixel is not None:
                            rp = (int(self.ref_pixel[0]), int(self.ref_pixel[1]))
                            cv2.circle(img, rp, 8, (0, 0, 255), 2)
                            cv2.putText(img, "ref", (rp[0] + 10, rp[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
                        if self.obj_bbox_point is not None:
                            cp = self.latest_detection_bundle.camera_pose.pose
                            R_cam_world = Rotation.from_quat([cp.orientation.x, cp.orientation.y,
                                                          cp.orientation.z, cp.orientation.w]).as_matrix()
                            T_cam_to_world_vis = np.eye(4)
                            T_cam_to_world_vis[:3, :3] = R_cam_world
                            T_cam_to_world_vis[:3, 3] = [cp.position.x, cp.position.y, cp.position.z]
                            T_world_to_cam = np.linalg.inv(T_cam_to_world_vis)
                            fx, fy = self.latest_detection_bundle.camera_info.k[0], self.latest_detection_bundle.camera_info.k[4]
                            cx2, cy2 = self.latest_detection_bundle.camera_info.k[2], self.latest_detection_bundle.camera_info.k[5]
                            p_cam = T_world_to_cam @ np.append(self.obj_bbox_point, 1.0)
                            if p_cam[2] > 0:
                                bu = int(fx * p_cam[0] / p_cam[2] + cx2)
                                bv = int(fy * p_cam[1] / p_cam[2] + cy2)
                                cv2.circle(img, (bu, bv), 6, (255, 0, 0), -1)
                                cv2.putText(img, "bbox", (bu + 10, bv), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)
                    if rgb_img.shape[:2] != depth_color.shape[:2]:
                        depth_color = cv2.resize(depth_color, (rgb_img.shape[1], rgb_img.shape[0]))
                    combined = np.hstack([rgb_img, depth_color])
                    # Resize combined to half desktop width (1920px)
                    target_w = 1920
                    target_h = int(combined.shape[0] * target_w / combined.shape[1])
                    combined = cv2.resize(combined, (target_w, target_h))
                    cv2.imshow("Detections", combined)
                    cv2.waitKey(1)
                else:
                    time.sleep(0.1)
        vis_thread = threading.Thread(target=visualize_detections_thread_func, daemon=True)
        vis_thread.start()


def print_menu():
    print("\n=== ARPA Core Control ===")
    print("1. Go home")
    print("2. Remove collision plane")
    print("3. Trigger behavior")
    print("4. Update depth")
    print("5. Motor control")
    print("6. Servo")
    print("7. Align to screw img")
    print("8. Remove Part Service")
    print("9. Motor test zforce then retract")
    print("10. Scan battery")
    print("0. Quit")
    print("========================")


def main(args=None):
    rclpy.init(args=args)
    node = CoreNode()
    node.visualize_detections()

    if node.use_collision_plane:
        node.add_collision_plane()

    try:
        while True:
            print_menu()
            choice = input("Select: ").strip()

            if choice == "1":
                print("\n=== What frame should be home ===")
                print("1. Wrist 3 link (default)")
                print("2. Toolhead")
                print("3. Camera")
                print("========================")
                frame_choice = input("Select: ").strip()
                frame_map = {
                    "1": "wrist_3_link",
                    "2": "test_ratchet_extension_link",
                    "3": "ee_cam_color_optical_frame",
                }
                frame_kwrd = frame_map.get(frame_choice, "wrist_3_link")
                success = node.go_home(frame_kwrd)
                print(f"GoHome result: {success}")

            elif choice == "2":
                plane_id = input("Plane ID [battery_do_not_cross]: ").strip() or "battery_do_not_cross"
                frame_id = input("Frame ID [floor_link]: ").strip() or "floor_link"
                node.remove_collision_plane(plane_id, frame_id)
                return
            elif choice == "3":
                behavior = input("Behavior name: ").strip()
                if behavior:
                    node.trigger_behavior(behavior)

            elif choice == "4":
                node.update_depth()

            elif choice == "5":
                speed = int(input("Speed (0=off): ").strip() or "0")
                node.motor_control(speed)

            elif choice == "6":
                print("sorry fam not implemented")
                # raw = input("x y z roll pitch yaw: ").strip().split()
                # v = np.array([float(n) for n in raw])
                # v = np.array([0.1, 0, 0, 0, 0, 0])
                # norm = np.linalg.norm(v)
                # if norm > 0:
                #     v = v / norm * 0.1
                # start_time = time.time()
                # while time.time() - start_time < 1:
                #     node.servo_twist(*v)
                #     time.sleep(0.02)  # ~50 Hz, well within 0.1s incoming_command_timeout
                # v = np.array([0,0,0,0,0,0])
                # node.servo_twist(*v)

            elif choice == "7":
                node.align_to_screw_img(None)
                # node.trigger_behavior("zforce")
                # node.trigger_behavior("play")
                # time.sleep(1)
                # node.trigger_behavior("retract")
                # node.trigger_behavior("play")
                # time.sleep(1)
                # node.trigger_behavior("ros2control")

            elif choice == "8":
                x, y, z, qx, qy, qz, qw = [1.026, -0.477, 0.857, -0.241, 0.971, 0.002, 0.000]
                req = RemovePart.Request()
                req = detection_confidence = 100.0
                req.target_pose.header.frame_id = BASE_FRAME
                req.visual_servo = True
                req.target_pose.pose.position.x = x
                req.target_pose.pose.position.y = y
                req.target_pose.pose.position.z = z
                req.target_pose.pose.orientation.x = qx
                req.target_pose.pose.orientation.y = qy
                req.target_pose.pose.orientation.z = qz
                req.target_pose.pose.orientation.w = qw
                future = node.remove_part_client.call_async(req)
                while not future.done():
                    time.sleep(0.05)
                result = future.result()
                print(f"RemovePart result: {result.success} — {result.message}")
            
            elif choice == "9":
                node.motor_control(100)
                node.trigger_behavior("zforce")
                node.trigger_behavior("play")
                time.sleep(5)
                node.trigger_behavior("retract")
                node.trigger_behavior("play")
                time.sleep(1)
                node.trigger_behavior("ros2control")
                node.motor_control(0)

            elif choice == "10":
                save_imgs = input("Save images? (y/n) [y]: ").strip().lower()
                save_images = save_imgs != "n"
                if ScanBattery is None:
                    print("ScanBattery action interface is unavailable in this workspace build.")
                    continue
                scan_client = ActionClient(node, ScanBattery, 'scan_battery')
                if not scan_client.wait_for_server(timeout_sec=5.0):
                    print("scan_battery action server not available")
                else:
                    goal = ScanBattery.Goal()
                    goal.save_images = save_images
                    future = scan_client.send_goal_async(
                        goal,
                        feedback_callback=lambda fb: print(
                            f"  Scan progress: {fb.feedback.points_explored}/{fb.feedback.total_points}"))
                    while not future.done():
                        time.sleep(0.05)
                    goal_handle = future.result()
                    if not goal_handle.accepted:
                        print("Goal rejected")
                    else:
                        print("Scan started — waiting for result...")
                        result_future = goal_handle.get_result_async()
                        while not result_future.done():
                            time.sleep(0.1)
                        r = result_future.result().result
                        print(f"Scan complete: success={r.success}, "
                              f"completed={r.completed}/{r.total_poses}, skipped={r.skipped}")

            elif choice == "0":
                break


            else:
                print("Invalid choice")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
