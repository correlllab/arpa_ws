#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import cv2
import json
import csv
import re
import time
import threading
from pathlib import Path
from cv_bridge import CvBridge

from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, JointState
import tf2_ros

BASE_FRAME = "floor_link"
EE_FRAME = "test_ratchet_extension_link"

# Placeholder — update to actual torque topic
TORQUE_TOPIC = "/motor_current"

# ── hardcoded trial metadata ──────────────────────────────────────────────────
REMOVAL_STRATEGY = "no_visual_servo"


class VLADataCollectionNode(Node):
    def __init__(self):
        super().__init__("vla_data_collection_node")

        self.declare_parameter("dataset_dir", "dataset")
        self._dataset_dir = Path.cwd() / "dataset"
        self._trials_dir = self._dataset_dir / "trials"
        self._trials_dir.mkdir(parents=True, exist_ok=True)
        self._metadata_path = self._dataset_dir / "metadata.csv"
        self._init_metadata_csv()

        self._cb_group = ReentrantCallbackGroup()
        self._bridge = CvBridge()
        self._lock = threading.Lock()

        # ── state ──
        self._trial_id = self._next_trial_id()
        self._fastener_name = ""
        self._removal_strategy = ""
        self._detection_confidence = ""

        self._latest_rgb: Image = None
        self._latest_depth: Image = None
        self._latest_joint_state: JointState = None

        self._recording = False
        self._torque_buf: list = []       # [(timestamp_ms, torque_Nm), ...]
        self._robot_state_buf: list = []  # [dict, ...]

        # ── servo loop state ──
        self._servo_active = False
        self._servo_step_idx = 0
        self._servo_start_time = 0.0
        self._servo_errors: list = []     # pixel error per step
        self._servo_converged = False

        # ── TF ──
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── subscribers ──
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image,
            "/realsense/ee_cam/color/image_raw",
            self._rgb_cb,
            img_qos,
            callback_group=self._cb_group,
        )
        # self.create_subscription(
        #     Image,
        #     "/realsense/ee_cam/aligned_depth_to_color/image_raw",
        #     self._depth_cb,
        #     img_qos,
        #     callback_group=self._cb_group,
        # )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Float32,
            TORQUE_TOPIC,
            self._wrench_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            String,
            "/triggered_behavior",
            self._behavior_cb,
            10,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"VLA data collection node ready — saving to {self._dataset_dir.resolve()}"
        )

    # ─────────────────────────── helpers ───────────────────────────

    def _init_metadata_csv(self):
        if not self._metadata_path.exists():
            with open(self._metadata_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "trial_id", "timestamp", "fastener_name",
                    "detection_confidence", "strategy",
                    "torque_peak", "label_engaged", "label_action",
                ])

    def _next_trial_id(self) -> int:
        existing = sorted(self._trials_dir.glob("trial_*"))
        if not existing:
            return 1
        return int(existing[-1].name.split("_")[1]) + 1

    def _trial_dir(self) -> Path:
        d = self._trials_dir / f"trial_{self._trial_id:04d}"
        d.mkdir(parents=True, exist_ok=True)
        return d

    def _stamp_to_ms(self, stamp) -> float:
        return stamp.sec * 1000.0 + stamp.nanosec / 1e6

    def _get_ee_pose(self) -> dict | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                BASE_FRAME, EE_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            t = tf.transform.translation
            r = tf.transform.rotation
            return {
                "position": {"x": t.x, "y": t.y, "z": t.z},
                "orientation": {"x": r.x, "y": r.y, "z": r.z, "w": r.w},
            }
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    # ─────────────────────────── topic callbacks ───────────────────

    def _rgb_cb(self, msg: Image):
        self._latest_rgb = msg

    def _depth_cb(self, msg: Image):
        self._latest_depth = msg

    def _joint_state_cb(self, msg: JointState):
        self._latest_joint_state = msg
        if self._recording:
            with self._lock:
                ee_pose = self._get_ee_pose()
                self._robot_state_buf.append(
                    {
                        "timestamp_ms": self._stamp_to_ms(msg.header.stamp),
                        "joint_names": list(msg.name),
                        "joint_positions": list(msg.position),
                        "joint_velocities": list(msg.velocity),
                        "joint_efforts": list(msg.effort),
                        "ee_pose": ee_pose,
                    }
                )

    def _wrench_cb(self, msg):
        if self._recording:
            ts = self._stamp_to_ms(self.get_clock().now().to_msg())
            torque_nm = msg.data
            with self._lock:
                self._torque_buf.append((ts, torque_nm))

    # ─────────────────────── behavior dispatch ─────────────────────

    _REMOVE_RE = re.compile(r"Remove: (.+?) with confidence = (.+)")

    def _behavior_cb(self, msg: String):
        text = msg.data.strip()

        m = self._REMOVE_RE.match(text)
        if m:
            self._fastener_name = m.group(1).strip()
            self._removal_strategy = REMOVAL_STRATEGY
            self._detection_confidence = float(m.group(2).strip())
            self.get_logger().info(
                f"New trial {self._trial_id:04d}: part={self._fastener_name}, "
                f"strategy={self._removal_strategy}, confidence={self._detection_confidence}"
            )
            return

        if text == "capture_images" or text.startswith("capture_images:"):
            tag = text[len("capture_images:"):].strip() if ":" in text else ""
            self._handle_capture_images(tag=tag)
        elif text == "start_recording":
            self._handle_start_recording()
        elif text == "stop_recording":
            self._handle_stop_recording()
        elif text == "start_servo_loop":
            self._handle_start_servo_loop()
        elif text.startswith("servo_step:"):
            self._handle_servo_step(text[len("servo_step:"):].strip())
        elif text == "stop_servo_loop":
            self._handle_stop_servo_loop(converged=True)
        elif text == "abort_servo_loop":
            self._handle_stop_servo_loop(converged=False)
        else:
            self.get_logger().debug(f"Unhandled behavior: {text}")

    # ─────────────────────── behaviour handlers ────────────────────

    def _handle_capture_images(self, tag: str = ""):
        trial_dir = self._trial_dir()
        suffix = f"_{tag}" if tag else ""

        if self._latest_rgb is not None:
            rgb = self._bridge.imgmsg_to_cv2(self._latest_rgb, desired_encoding="bgr8")
            fname = f"rgb{suffix}.png"
            cv2.imwrite(str(trial_dir / fname), rgb)
            self.get_logger().info(f"Saved {fname} for trial {self._trial_id:04d}")
        else:
            self.get_logger().warn("No RGB image available for capture")

        # if self._latest_depth is not None:
        #     depth = self._bridge.imgmsg_to_cv2(self._latest_depth, desired_encoding="passthrough")
        #     fname = f"depth{suffix}.png"
        #     cv2.imwrite(str(trial_dir / fname), depth.astype(np.uint16))
        #     self.get_logger().info(f"Saved {fname} for trial {self._trial_id:04d}")
        # else:
        #     self.get_logger().warn("No depth image available for capture")

    def _handle_start_recording(self):
        with self._lock:
            self._torque_buf.clear()
            self._robot_state_buf.clear()
            self._recording = True
        self.get_logger().info(f"Recording started for trial {self._trial_id:04d}")

    def _handle_stop_recording(self):
        with self._lock:
            self._recording = False
            torque_data = list(self._torque_buf)
            robot_state_data = list(self._robot_state_buf)

        trial_dir = self._trial_dir()

        # torque.csv
        torque_peak = ""
        if torque_data:
            with open(trial_dir / "torque.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp_ms", "torque_Nm"])
                writer.writerows(torque_data)
            torques = np.array(torque_data, dtype=np.float64)[:, 1]
            torque_peak = f"{np.max(np.abs(torques)):.4f}"
            self.get_logger().info(
                f"Saved torque.csv ({len(torque_data)} samples) for trial {self._trial_id:04d}"
            )
        else:
            self.get_logger().warn(f"No torque data recorded for trial {self._trial_id:04d}")

        # joints.csv and ee_pose.csv
        if robot_state_data:
            # joints.csv — one row per joint-state snapshot
            joint_names = robot_state_data[0]["joint_names"]
            with open(trial_dir / "joints.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp_ms"] + joint_names +
                                [f"vel_{n}" for n in joint_names] +
                                [f"eff_{n}" for n in joint_names])
                for snap in robot_state_data:
                    writer.writerow(
                        [snap["timestamp_ms"]] +
                        list(snap["joint_positions"]) +
                        list(snap["joint_velocities"]) +
                        list(snap["joint_efforts"])
                    )
            self.get_logger().info(
                f"Saved joints.csv ({len(robot_state_data)} snapshots) for trial {self._trial_id:04d}"
            )

            # ee_pose.csv — one row per snapshot that has a valid EE pose
            with open(trial_dir / "ee_pose.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp_ms", "x", "y", "z", "qx", "qy", "qz", "qw"])
                for snap in robot_state_data:
                    ep = snap.get("ee_pose")
                    if ep:
                        p = ep["position"]
                        o = ep["orientation"]
                        writer.writerow([
                            snap["timestamp_ms"],
                            p["x"], p["y"], p["z"],
                            o["x"], o["y"], o["z"], o["w"],
                        ])
            self.get_logger().info(f"Saved ee_pose.csv for trial {self._trial_id:04d}")
        else:
            self.get_logger().warn(f"No robot state recorded for trial {self._trial_id:04d}")

        # metadata row
        with open(self._metadata_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                f"trial_{self._trial_id:04d}",
                time.strftime("%Y-%m-%dT%H:%M:%S"),
                self._fastener_name,
                self._detection_confidence,
                self._removal_strategy,
                torque_peak,
                "",  # label_engaged — filled post-hoc
                "",  # label_action — filled post-hoc
            ])

        self.get_logger().info(f"Trial {self._trial_id:04d} complete — advancing to next trial")
        self._trial_id += 1
        self._fastener_name = ""
        self._servo_errors.clear()
        self._servo_converged = False

    # ─────────────────────── servo loop handlers ───────────────────

    def _handle_start_servo_loop(self):
        trial_dir = self._trial_dir()
        servo_dir = trial_dir / "servo_loop"
        servo_dir.mkdir(parents=True, exist_ok=True)

        self._servo_active = True
        self._servo_step_idx = 0
        self._servo_start_time = time.monotonic()
        self._servo_errors.clear()
        self._servo_converged = False
        self.get_logger().info(f"Servo loop started for trial {self._trial_id:04d}")

    def _handle_servo_step(self, payload: str):
        """Expects JSON: {"pixel_error": float, "depth_Z": float, "velocity_cmd": [x,y,z]}"""
        if not self._servo_active:
            self.get_logger().warn("servo_step received but no servo loop is active")
            return

        try:
            data = json.loads(payload)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid servo_step JSON: {e}")
            return

        trial_dir = self._trial_dir()
        servo_dir = trial_dir / "servo_loop"
        idx = self._servo_step_idx

        # Save step RGB
        if self._latest_rgb is not None:
            rgb = self._bridge.imgmsg_to_cv2(self._latest_rgb, desired_encoding="bgr8")
            cv2.imwrite(str(servo_dir / f"step_{idx:02d}_rgb.png"), rgb)

        # Record EE pose for this step
        ee_pose = self._get_ee_pose()
        step_record = {
            "pixel_error": data.get("pixel_error"),
            "depth_Z": data.get("depth_Z"),
            "velocity_cmd": data.get("velocity_cmd"),
            "ee_pose": ee_pose,
        }
        with open(servo_dir / f"step_{idx:02d}.json", "w") as f:
            json.dump(step_record, f, indent=2)

        pixel_error = data.get("pixel_error", 0.0)
        self._servo_errors.append(pixel_error)
        self._servo_step_idx += 1

        self.get_logger().debug(
            f"Servo step {idx}: pixel_error={pixel_error:.2f}"
        )

    def _handle_stop_servo_loop(self, converged: bool):
        if not self._servo_active:
            self.get_logger().warn("stop_servo_loop received but no servo loop is active")
            return

        self._servo_active = False
        self._servo_converged = converged
        elapsed = time.monotonic() - self._servo_start_time

        trial_dir = self._trial_dir()
        servo_dir = trial_dir / "servo_loop"

        summary = {
            "n_steps": self._servo_step_idx,
            "converged": converged,
            "final_error": self._servo_errors[-1] if self._servo_errors else None,
            "total_time_s": round(elapsed, 3),
        }
        with open(servo_dir / "summary.json", "w") as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info(
            f"Servo loop ended for trial {self._trial_id:04d}: "
            f"{self._servo_step_idx} steps, converged={converged}, "
            f"final_error={summary['final_error']}, time={elapsed:.2f}s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VLADataCollectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
