#!/usr/bin/env python3
"""Publish zero joint states for a list of humanoid joints."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ZeroJointStatePublisher(Node):
    def __init__(self):
        super().__init__("humanoid_zero_joint_state_publisher")
        self.declare_parameter("joint_names_csv", "")
        self.declare_parameter("topic_name", "/joint_states")
        joint_names_csv = self.get_parameter("joint_names_csv").value
        self._joint_names = [name for name in joint_names_csv.split(",") if name]
        topic_name = self.get_parameter("topic_name").value
        self._pub = self.create_publisher(JointState, topic_name, 10)
        self._timer = self.create_timer(0.1, self._publish)
        self.get_logger().info(
            f"Publishing zero joint states for {len(self._joint_names)} joints on {topic_name}"
        )

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = [0.0] * len(self._joint_names)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZeroJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
