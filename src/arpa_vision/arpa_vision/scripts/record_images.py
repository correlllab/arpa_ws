"""
record_images.py
Subscribes to the EE camera RGB topic. Press Enter to save the latest frame as a PNG.
Usage:
    ros2 run arpa_vision record_images
    ros2 run arpa_vision record_images --ros-args -p dataset_folder:=/path/to/folder
"""
import os
import threading
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

rgb_topic = "/realsense/ee_cam/color/image_raw/compressed"


class RecordImagesNode(Node):
    def __init__(self):
        super().__init__('record_images')

        default_folder = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'dataset')
        self.declare_parameter('dataset_folder', default_folder)
        self.dataset_folder = self.get_parameter('dataset_folder').get_parameter_value().string_value
        os.makedirs(self.dataset_folder, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.sub = self.create_subscription(CompressedImage, rgb_topic, self._cb, qos)
        self.get_logger().info(f'Subscribed to {rgb_topic}')
        self.get_logger().info(f'Saving images to: {self.dataset_folder}')
        self.get_logger().info('Press Enter to save a frame.')

    def _cb(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.latest_frame = frame

    def save_latest(self):
        with self.lock:
            frame = self.latest_frame.copy() if self.latest_frame is not None else None
        if frame is None:
            self.get_logger().warn('No frame received yet.')
            return
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.dataset_folder, f'{timestamp}.png')
        cv2.imwrite(filename, frame)
        self.get_logger().info(f'Saved: {filename}')


def input_loop(node):
    while rclpy.ok():
        try:
            input()  # blocks until Enter
        except EOFError:
            break
        node.save_latest()


def main(args=None):
    rclpy.init(args=args)
    node = RecordImagesNode()

    spin_thread  = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    input_thread = threading.Thread(target=input_loop, args=(node,), daemon=True)
    spin_thread.start()
    input_thread.start()

    cv2.namedWindow('Record Images', cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            with node.lock:
                frame = node.latest_frame.copy() if node.latest_frame is not None else None
            if frame is not None:
                cv2.imshow('Record Images', frame)
            if cv2.waitKey(33) == ord('q'):  # ~30 fps; q to quit
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
