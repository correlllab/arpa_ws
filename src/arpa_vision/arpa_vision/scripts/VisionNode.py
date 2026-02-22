import sys
import os
import queue
import threading
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
import message_filters
import tf2_ros
import open3d as o3d
import open3d.t.geometry as o3tg
import open3d.core as o3c
import torch


sys.path.insert(0, os.path.dirname(__file__))
from BoundingBoxDetectors import YOLO_WORLD
from SAM2_WRAPPER import sam2

rgb_topic         = "/realsense/ee_cam/color/image_raw/compressed"
depth_topic       = "/realsense/ee_cam/aligned_depth_to_color/image_raw"
camera_info_topic = "/realsense/ee_cam/color/camera_info"
annotated_topic   = "/realsense/ee_cam/image_annotated"
markers_topic     = "/realsense/ee_cam/object_markers"
cloud_topic       = "/realsense/ee_cam/object_pointcloud"

BASE_FRAME = "world"
OVERLAP_THRESHOLD = 0.10  # 10% minimum overlap to match existing detection
VOXEL_SIZE_M = 0.0001


COLORS = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255)]
MARKER_COLORS = [
    ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5),
    ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5),
    ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5),
    ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5),
    ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.5),
]

WeightFilePath = os.path.join(os.path.dirname(__file__), "YoloWorldL.pth")
assert os.path.exists(WeightFilePath), f"Weight file {WeightFilePath} does not exist DUMMY"
print(f"Using weight file at: {WeightFilePath}")
QUERIES = [
    "Bolt",
    "BusBar",
    "InteriorScrew",
    "Nut",
    "OrangeCover",
    "Screw",
    "Screw Hole"
]


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.bridge = CvBridge()
        self.detector = YOLO_WORLD(WeightFilePath)
        self.segmentor = sam2()
        self.sync_queue = queue.Queue()
        self.lock = threading.Lock()

        # Confirm CUDA usage
        yolo_device = next(self.detector.model.model.parameters()).device
        sam_device  = self.segmentor.device
        self.get_logger().info(f'YOLO_WORLD device: {yolo_device}')
        self.get_logger().info(f'SAM2 device:       {sam_device}')

        # Persistent detections: list of {'pcd', 'bbox', 'label', 'prob'}
        self.detections = []
        self.last_header = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        rgb_sub   = message_filters.Subscriber(self, CompressedImage, rgb_topic,          qos_profile=qos)
        depth_sub = message_filters.Subscriber(self, Image,           depth_topic,         qos_profile=qos)
        info_sub  = message_filters.Subscriber(self, CameraInfo,      camera_info_topic,   qos_profile=qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            queue_size=10,
            slop=0.05,
        )
        self.sync.registerCallback(self.sync_callback)

        self.pub         = self.create_publisher(Image,       annotated_topic, qos)
        self.markers_pub = self.create_publisher(MarkerArray, markers_topic,   10)
        self.cloud_pub   = self.create_publisher(PointCloud2, cloud_topic,     10)

        self.get_logger().info(f'VisionNode started. Detecting: {QUERIES}')

    def sync_callback(self, rgb_msg, depth_msg, info_msg):
        with self.lock:
            self.sync_queue.put((rgb_msg, depth_msg, info_msg))

    def process_queue(self):
        while True:
            with self.lock:
                if self.sync_queue.empty():
                    break
                rgb_msg, depth_msg, info_msg = self.sync_queue.get()

            # Drop stale frames so we never request a TF timestamp older than the buffer
            msg_age = (self.get_clock().now() - rclpy.time.Time.from_msg(rgb_msg.header.stamp)).nanoseconds * 1e-9
            if msg_age > 0.5:
                self.get_logger().warn(f'Dropping stale frame ({msg_age:.2f}s old)')
                continue

            try:
                transform = self.tf_buffer.lookup_transform(
                    BASE_FRAME,
                    rgb_msg.header.frame_id,
                    rgb_msg.header.stamp,
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'TF lookup failed: {e}')
                continue

            img       = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_m   = (depth_raw / 1000.0).astype('float32')  # uint16 mm → float32 m

            intrinsics = {
                "fx": info_msg.k[0], "fy": info_msg.k[4],
                "cx": info_msg.k[2], "cy": info_msg.k[5],
            }
            obs_pose = self._transform_to_matrix(transform)

            candidates = self.detector.predict(img, QUERIES, debug=False)
            annotated  = self._draw_boxes(img, candidates)
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = rgb_msg.header
            self.pub.publish(out_msg)

            # Run SAM2 and accumulate point clouds
            all_boxes = []
            all_probs = []
            box_labels = []
            for label, pred in candidates.items():
                for box, prob in zip(pred['boxes'], pred['probs']):
                    all_boxes.append(box)
                    all_probs.append(prob)
                    box_labels.append(label)

            if all_boxes:
                pcds, bboxes_3d, probs_3d, _, _ = self.segmentor.predict(
                    rgb_img=img,
                    depth_img=depth_m,
                    bbox=np.array(all_boxes),
                    probs=all_probs,
                    intrinsics=intrinsics,
                    obs_pose=obs_pose,
                    debug=False,
                )
                self._update_detections(pcds, bboxes_3d, box_labels[:len(pcds)], probs_3d)

            self.last_header = rgb_msg.header

    def _update_detections(self, pcds, bboxes_3d, labels, probs):
        """
        For each new detection, check if its bounding box overlaps an existing one
        by at least OVERLAP_THRESHOLD. If so, merge the PCDs. Otherwise add as new.
        """
        for pcd, bbox, label, prob in zip(pcds, bboxes_3d, labels, probs):
            matched = False
            for det in self.detections:
                if self._bbox_overlap_ratio(bbox, det['bbox']) >= OVERLAP_THRESHOLD:
                    det['pcd']  = self._merge_pcds(det['pcd'], pcd)
                    det['bbox'] = det['pcd'].get_axis_aligned_bounding_box()
                    det['prob'] = max(det['prob'], prob)
                    matched = True
                    break
            if not matched:
                self.detections.append({
                    'pcd':   pcd,
                    'bbox':  bbox,
                    'label': label,
                    'prob':  prob,
                })

    def _bbox_overlap_ratio(self, bbox1, bbox2):
        """Intersection volume / min(vol1, vol2) for two AxisAlignedBoundingBoxes."""
        mn1 = bbox1.min_bound.numpy()
        mx1 = bbox1.max_bound.numpy()
        mn2 = bbox2.min_bound.numpy()
        mx2 = bbox2.max_bound.numpy()

        inter_min = np.maximum(mn1, mn2)
        inter_max = np.minimum(mx1, mx2)
        inter_vol = np.prod(np.maximum(0.0, inter_max - inter_min))

        vol1 = np.prod(np.maximum(0.0, mx1 - mn1))
        vol2 = np.prod(np.maximum(0.0, mx2 - mn2))
        min_vol = min(vol1, vol2)
        if min_vol <= 0.0:
            return 0.0
        return inter_vol / min_vol

    def _merge_pcds(self, pcd1, pcd2):
        """Stack two tensor PointClouds and voxel-downsample the result."""
        pts = np.vstack([pcd1.point["positions"].numpy(), pcd2.point["positions"].numpy()])
        cls = np.vstack([pcd1.point["colors"].numpy(),    pcd2.point["colors"].numpy()])
        merged = o3tg.PointCloud()
        merged.point["positions"] = o3c.Tensor(pts, o3c.float32)
        merged.point["colors"]    = o3c.Tensor(cls, o3c.float32)
        return merged.voxel_down_sample(voxel_size=VOXEL_SIZE_M)

    def publish_detections(self):
        """Publish MarkerArray and merged PointCloud2 from all accumulated detections."""
        if not self.detections or self.last_header is None:
            return

        # MarkerArray — one CUBE per detection
        markers = MarkerArray()
        for i, det in enumerate(self.detections):
            mn = det['bbox'].min_bound.numpy()
            mx = det['bbox'].max_bound.numpy()
            center = (mn + mx) / 2.0
            size   = mx - mn

            m = Marker()
            m.header          = self.last_header
            m.header.frame_id = BASE_FRAME
            m.ns              = det['label']
            m.id              = i
            m.type            = Marker.CUBE
            m.action          = Marker.ADD
            m.pose.position.x = float(center[0])
            m.pose.position.y = float(center[1])
            m.pose.position.z = float(center[2])
            m.pose.orientation.w = 1.0
            m.scale.x = float(max(size[0], 0.005))
            m.scale.y = float(max(size[1], 0.005))
            m.scale.z = float(max(size[2], 0.005))
            m.color   = MARKER_COLORS[i % len(MARKER_COLORS)]
            markers.markers.append(m)
        self.markers_pub.publish(markers)

        # PointCloud2 — all detection PCDs merged into one message
        all_pts = np.vstack([d['pcd'].point["positions"].numpy() for d in self.detections])
        all_cls = np.vstack([d['pcd'].point["colors"].numpy()    for d in self.detections])
        data = np.hstack([all_pts, all_cls]).astype(np.float32)

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='g', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='b', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = PointCloud2()
        cloud_msg.header          = self.last_header
        cloud_msg.header.frame_id = BASE_FRAME
        cloud_msg.height          = 1
        cloud_msg.width           = len(all_pts)
        cloud_msg.fields          = fields
        cloud_msg.is_bigendian    = False
        cloud_msg.point_step      = 24
        cloud_msg.row_step        = 24 * len(all_pts)
        cloud_msg.data            = data.tobytes()
        cloud_msg.is_dense        = True
        self.cloud_pub.publish(cloud_msg)

    def _transform_to_matrix(self, transform):
        from scipy.spatial.transform import Rotation
        t = transform.transform.translation
        r = transform.transform.rotation
        rot = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = rot
        mat[:3,  3] = [t.x, t.y, t.z]
        return mat

    def _draw_boxes(self, img, candidates):
        out = img.copy()
        for i, (label, pred) in enumerate(candidates.items()):
            color = COLORS[i % len(COLORS)]
            for box, prob in zip(pred['boxes'], pred['probs']):
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
                cv2.putText(out, f'{label} {prob:.2f}', (x1, max(y1 - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            node.process_queue()
            node.publish_detections()
            time.sleep(0.01)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
