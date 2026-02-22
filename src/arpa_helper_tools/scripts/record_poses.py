#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

SOURCE_FRAME = "world"
TARGET_FRAME = "wrist_3_link"


class RecordPoseNode(Node):
    def __init__(self):
        super().__init__('record_pose_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Store recorded poses for visualization
        self.recorded_poses = []

        # Publisher for marker visualization
        self.marker_pub = self.create_publisher(MarkerArray, 'recorded_poses_markers', 10)
        self.marker_timer = self.create_timer(0.1, self.publish_markers)

        # Setup output file (save to src/scripts, not install/lib)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        if '/install/' in script_dir:
            script_dir = script_dir.replace('/install/', '/src/')
            script_dir = script_dir.replace('/lib/arpa_helper_tools', '/scripts')
        self.filename = os.path.join(script_dir, f"{SOURCE_FRAME}_to_{TARGET_FRAME}.txt")

        if os.path.exists(self.filename):
            self.load_poses_from_file()
            self.get_logger().info(f"Loaded {len(self.recorded_poses)} poses from {self.filename}")
        else:
            with open(self.filename, 'w') as f:
                f.write(f"# Poses from {SOURCE_FRAME} to {TARGET_FRAME}\n")
                f.write("# x,y,z,qx,qy,qz,qw\n")
            self.get_logger().info(f"Created pose file: {self.filename}")

        self.get_logger().info("Press Enter to record a pose, 'q' to quit")

    def load_poses_from_file(self):
        with open(self.filename, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                try:
                    values = [float(v) for v in line.split(',')]
                    if len(values) == 7:
                        self.recorded_poses.append(tuple(values))
                except ValueError:
                    continue

    def record_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                SOURCE_FRAME,
                TARGET_FRAME,
                rclpy.time.Time()
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            pose_str = f"{t.x},{t.y},{t.z},{r.x},{r.y},{r.z},{r.w}"

            with open(self.filename, 'a') as f:
                f.write(pose_str + "\n")

            # Store pose for visualization
            self.recorded_poses.append((t.x, t.y, t.z, r.x, r.y, r.z, r.w))

            self.get_logger().info(f"Recorded: {pose_str}")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return False

    def publish_markers(self):
        marker_array = MarkerArray()

        for i, pose in enumerate(self.recorded_poses):
            marker = Marker()
            marker.header.frame_id = SOURCE_FRAME
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "recorded_poses"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            marker.pose.position.z = pose[2]
            marker.pose.orientation.x = pose[3]
            marker.pose.orientation.y = pose[4]
            marker.pose.orientation.z = pose[5]
            marker.pose.orientation.w = pose[6]

            marker.scale.x = 0.1   # shaft length
            marker.scale.y = 0.02  # shaft diameter
            marker.scale.z = 0.02  # head diameter

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    import threading

    rclpy.init(args=args)
    node = RecordPoseNode()

    # Spin in a separate thread so timer runs
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            user_input = input()
            if user_input.lower() == 'q':
                break
            node.record_pose()

    except (KeyboardInterrupt, EOFError):
        pass

    node.get_logger().info(f"Poses saved to {node.filename}")
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
