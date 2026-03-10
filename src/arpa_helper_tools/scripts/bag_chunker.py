#!/usr/bin/env python3
"""
Node that chunks a ROS2 bag into smaller bags based on /triggered_behavior messages.

Starts recording a new chunk when "second_sight" is received, and stops/saves
when "retracted_finishes" is received. Each chunk is saved as <bag_name>_<attempt>.
"""

import subprocess
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BagChunker(Node):
    def __init__(self):
        super().__init__('bag_chunker')

        self.declare_parameter('input_bag', '')
        self.declare_parameter('output_dir', '.')

        self.input_bag = self.get_parameter('input_bag').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value

        if not self.input_bag:
            self.get_logger().error('input_bag parameter is required')
            raise SystemExit(1)

        self.attempt_number = 0
        self.recording_process = None
        self.topics = self._get_bag_topics()

        if not self.topics:
            self.get_logger().error(f'No topics found in bag: {self.input_bag}')
            raise SystemExit(1)

        self.get_logger().info(f'Found {len(self.topics)} topics in bag')
        self.get_logger().info(f'Listening for triggers on /triggered_behavior')

        self.subscription = self.create_subscription(
            String,
            '/triggered_behavior',
            self.trigger_callback,
            10
        )

    def _get_bag_topics(self):
        try:
            result = subprocess.run(
                ['ros2', 'bag', 'info', self.input_bag],
                capture_output=True, text=True, timeout=30
            )
            topics = []
            in_topic_section = False
            for line in result.stdout.splitlines():
                stripped = line.strip()
                if stripped.startswith('Topic:'):
                    # Single topic info line format: "Topic: /topic_name | Type: ... | Count: ..."
                    topic = stripped.split('|')[0].replace('Topic:', '').strip()
                    if topic:
                        topics.append(topic)
                elif 'Topic information:' in stripped:
                    in_topic_section = True
                elif in_topic_section and stripped:
                    # Format: "Topic: /topic  | Type: type | Count: N"
                    if stripped.startswith('Topic:'):
                        topic = stripped.split('|')[0].replace('Topic:', '').strip()
                        if topic:
                            topics.append(topic)
            return topics
        except Exception as e:
            self.get_logger().error(f'Failed to read bag info: {e}')
            return []

    def trigger_callback(self, msg):
        value = msg.data.strip()

        if value == 'second_sight':
            self._start_recording()
        elif value == 'retracted_finishes':
            self._stop_recording()

    def _start_recording(self):
        if self.recording_process is not None:
            self.get_logger().warn('Already recording, stopping current before starting new')
            self._stop_recording()

        self.attempt_number += 1
        bag_name = f'{self.input_bag}_{self.attempt_number}'
        output_path = f'{self.output_dir}/{bag_name}' if self.output_dir != '.' else bag_name

        cmd = ['ros2', 'bag', 'record', '-o', output_path]
        for topic in self.topics:
            cmd.append(topic)

        self.get_logger().info(f'Starting recording attempt {self.attempt_number}: {output_path}')
        self.recording_process = subprocess.Popen(cmd)

    def _stop_recording(self):
        if self.recording_process is None:
            self.get_logger().warn('No active recording to stop')
            return

        self.get_logger().info(f'Stopping recording attempt {self.attempt_number}')
        self.recording_process.send_signal(signal.SIGINT)
        try:
            self.recording_process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Recording process did not stop cleanly, killing')
            self.recording_process.kill()
            self.recording_process.wait()
        self.recording_process = None
        self.get_logger().info(f'Saved attempt {self.attempt_number}')

    def destroy_node(self):
        if self.recording_process is not None:
            self._stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BagChunker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
