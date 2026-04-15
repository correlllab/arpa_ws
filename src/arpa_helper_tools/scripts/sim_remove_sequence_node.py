#!/usr/bin/env python3
"""
Sim Remove Sequence node: iterates all parts in the parts list,
calling /remove_part service for each part.
"""

import os
import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from custom_ros_messages.srv import RemovePart
from ament_index_python.packages import get_package_share_directory

FRAME_ID = "floor_link"
SERVICE_WAIT_SEC = 10.0
REMOVE_PART_TIMEOUT_SEC = 120.0


class SimRemoveSequenceNode(Node):
    def __init__(self):
        super().__init__('sim_remove_sequence_node')

        self._cb_group = ReentrantCallbackGroup()

        self._remove_part_client = self.create_client(
            RemovePart, '/remove_part', callback_group=self._cb_group
        )
        self._progress_pub = self.create_publisher(String, '/triggered_behavior', 10)

        self._parts = self._load_parts()
        self.get_logger().info(
            f'sim_remove_sequence_node ready: {len(self._parts)} parts loaded.'
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

    def run_sequence(self):
        if not self._parts:
            self.get_logger().error('No parts loaded')
            return

        if not self._remove_part_client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            self.get_logger().error(
                f'/remove_part service not available (waited {SERVICE_WAIT_SEC:.0f}s)'
            )
            return

        n = len(self._parts)
        parts_list = list(self._parts.items())
        run = 0

        while rclpy.ok():
            run += 1
            failed = []
            self._publish_progress(f'Sim Remove: Starting run {run}')

            for i, (name, coords) in enumerate(parts_list):
                part_num = i + 1
                x, y, z_raw = coords

                target_pose = PoseStamped()
                target_pose.header.frame_id = FRAME_ID
                target_pose.header.stamp.sec = 0
                target_pose.header.stamp.nanosec = 0
                target_pose.pose.position.x = x
                target_pose.pose.position.y = y
                target_pose.pose.position.z = z_raw
                # orientation.w=1.0 signals core to compute tool-down quaternion from x,y
                target_pose.pose.orientation.w = 1.0

                req = RemovePart.Request()
                req.part_name = name
                req.target_pose = target_pose
                req.visual_servo = False
                req.detection_confidence = 0.0

                self._publish_progress(
                    f'Sim Remove: Part {part_num}/{n} ({name}) — calling /remove_part'
                )

                done_event = threading.Event()
                future = self._remove_part_client.call_async(req)
                future.add_done_callback(lambda _: done_event.set())
                completed = done_event.wait(timeout=REMOVE_PART_TIMEOUT_SEC)

                if not completed or not future.done():
                    self.get_logger().warn(f'Part {part_num}/{n} ({name}) timed out')
                    self._publish_progress(
                        f'Sim Remove: Part {part_num}/{n} ({name}) TIMED OUT — skipping'
                    )
                    failed.append(name)
                    continue

                result = future.result()
                if result.success:
                    self._publish_progress(
                        f'Sim Remove: Part {part_num}/{n} ({name}) — done'
                    )
                else:
                    self.get_logger().warn(
                        f'Part {part_num}/{n} ({name}) failed: {result.message}'
                    )
                    self._publish_progress(
                        f'Sim Remove: Part {part_num}/{n} ({name}) FAILED — skipping'
                    )
                    failed.append(name)

            if failed:
                msg = (
                    f'Run {run} done. {n - len(failed)}/{n} succeeded. '
                    f'Failed: {", ".join(failed)}'
                )
            else:
                msg = f'Run {run} complete. All {n} parts done.'

            self._publish_progress(f'Sim Remove: {msg}')


def main(args=None):
    rclpy.init(args=args)
    node = SimRemoveSequenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
