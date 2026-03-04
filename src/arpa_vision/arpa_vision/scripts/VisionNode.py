import sys
import os
os.chdir(os.path.expanduser('~'))  # PyTorch import crashes if CWD doesn't exist (ros2 run quirk)
import queue
import threading
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, PointCloud2, PointField  # Image kept for annotated publisher
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
import message_filters
import pickle
import json
import tf2_ros
import open3d.t.geometry as o3tg
import open3d.core as o3c
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from BoundingBoxDetectors import YOLO_WORLD

rgb_topic         = "/realsense/ee_cam/color/image_raw/compressed"
depth_topic       = "/realsense/ee_cam/aligned_depth_to_color/image_raw/compressedDepth"
camera_info_topic = "/realsense/ee_cam/color/camera_info"
annotated_topic   = "/realsense/ee_cam/image_annotated"
markers_topic     = "/realsense/ee_cam/object_markers"
cloud_topic       = "/realsense/ee_cam/object_pointcloud"

BASE_FRAME         = "world"
OVERLAP_THRESHOLD  = 0.01
DO_VOXEL           = True
VOXEL_SIZE_M       = 0.001
CAMERA_MIN_RANGE_M = 0.1
CAMERA_MAX_RANGE_M = 0.5

# --- Outlier removal ---
RADIUS_OUTLIER_REMOVAL        = True
RADIUS_OUTLIER_NB_POINTS      = 64     # min neighbours within radius
RADIUS_OUTLIER_RADIUS         = 0.01  # search radius in metres

STATISTICAL_OUTLIER_REMOVAL      = True
STATISTICAL_OUTLIER_NB_NEIGHBORS = 64  # neighbours to analyse
STATISTICAL_OUTLIER_STD_RATIO    = 0.05 # std-dev multiplier threshold

PCD_MIN_POINTS = 100  # discard clouds with fewer points than this

BLUR_THRESHOLD = 0.0  # Laplacian variance below this → image is too blurry

SUBSCRIBER_RATE_HZ = 6.0
PUBLISHER_RATE_HZ = 6.0

COLORS = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255)]
MARKER_COLORS = [ColorRGBA(r=float(r)/255.0, g=float(g)/255.0, b=float(b)/255.0, a=0.1) for r, g, b in COLORS]

WeightFilePath = os.path.join(os.path.dirname(os.path.realpath(__file__)), "yolov8x-worldv2_best.pt")
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

        self.bridge   = CvBridge()
        self.detector = YOLO_WORLD(WeightFilePath)
        self.sync_queue = queue.Queue()
        self.lock = threading.Lock()

        yolo_device = next(self.detector.model.model.parameters()).device
        self.get_logger().info(f'YOLO_WORLD device: {yolo_device}')

        # Persistent detections: dict of {label: 'pcd', 'bbox', 'prob'}
        self.detections = {}
        self.last_header = None
        self.last_annotated = None

        self._last_sync_time = 0.0
        self._sync_interval = 1.0 / SUBSCRIBER_RATE_HZ

        self._last_publish_time = 0.0
        self._publish_interval = 1.0 / PUBLISHER_RATE_HZ

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        rgb_sub   = message_filters.Subscriber(self, CompressedImage, rgb_topic,        qos_profile=qos)
        depth_sub = message_filters.Subscriber(self, CompressedImage, depth_topic,       qos_profile=qos)
        info_sub  = message_filters.Subscriber(self, CameraInfo,      camera_info_topic, qos_profile=qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            queue_size=10,
            slop=0.1,
        )
        self.sync.registerCallback(self.sync_callback)

        self.annotated_pub         = self.create_publisher(Image,       annotated_topic, qos)
        self.markers_pub = self.create_publisher(MarkerArray, markers_topic,   10)
        self.cloud_pub   = self.create_publisher(PointCloud2, cloud_topic,     10)

        _default_save_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'object_detections.pkl')
        self.declare_parameter('detections_save_path', _default_save_path)
        self.create_service(Trigger, '~/save_object_detection', self._save_detections_cb)
        self.create_service(Trigger, '~/load_object_detection', self._load_detections_cb)
        self.create_service(Trigger, '~/get_detections_json',   self._get_detections_json_cb)
        self.create_service(Trigger, '~/clear_detections',      self._clear_detections_cb)

        _save_path = self.get_parameter('detections_save_path').get_parameter_value().string_value
        if os.path.exists(_save_path):
            self._load_detections(_save_path)
            # pass
        else:
            self.get_logger().info(f'No saved detections found at {_save_path}, starting fresh.')

        self.get_logger().info(f'VisionNode started. Detecting: {QUERIES}')

    def sync_callback(self, rgb_msg, depth_msg, info_msg):
        # self.get_logger().info(f'adding to sync queue...')
        
        now = time.time()
        if now - self._last_sync_time < self._sync_interval:
            # self.get_logger().info(f'canceling addition to sync queue (freq)...')
            return
        self._last_sync_time = now

        with self.lock:
            self.sync_queue.put((rgb_msg, depth_msg, info_msg))
        # self.get_logger().info(f'successfully added to sync queue...')

    def process_queue(self):
        # self.get_logger().info(f'Processing sync queue...')
        rgb_msg, depth_msg, info_msg = None, None, None
        with self.lock:
            if self.sync_queue.empty():
                # self.get_logger().info(f'No synchronized messages to process')
                return
            # Drain to the newest frame — older ones will just be stale
            while not self.sync_queue.empty():
                rgb_msg, depth_msg, info_msg = self.sync_queue.get()

        img = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        blur_score = cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
        if blur_score < BLUR_THRESHOLD:
            self.get_logger().warn(f'Dropping blurry frame (Laplacian var={blur_score:.1f})')
            return

        # Drop stale frames so we never request a TF timestamp older than the buffer
        msg_age = (self.get_clock().now() - rclpy.time.Time.from_msg(rgb_msg.header.stamp)).nanoseconds * 1e-9
        if msg_age > 5.0:
            self.get_logger().warn(f'Dropping stale frame ({msg_age:.2f}s old)')
            return

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
            return

        img = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')

        # compressedDepth has a 12-byte config header before the PNG payload
        depth_buf = np.frombuffer(bytes(depth_msg.data)[12:], dtype=np.uint8)
        depth_raw = cv2.imdecode(depth_buf, cv2.IMREAD_UNCHANGED)  # uint16 mm
        depth_m   = (depth_raw / 1000.0).astype('float32')

        intrinsics = {
            "fx": info_msg.k[0], "fy": info_msg.k[4],
            "cx": info_msg.k[2], "cy": info_msg.k[5],
        }
        obs_pose = self._transform_to_matrix(transform)

        t0 = time.time()
        candidates = self.detector.predict(img, QUERIES, debug=False)
        yolo_ms = (time.time() - t0) * 1000

        annotated = self._draw_boxes(img, candidates)
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = rgb_msg.header
        self.last_annotated = out_msg
        # self.get_logger().info(f'last annotated set')


        # Back-project each bounding box region to a 3D point cloud
        t1 = time.time()
        pcds, bboxes_3d, labels_3d, probs_3d = [], [], [], []
        img_area = img.shape[0] * img.shape[1]
        for label, pred in candidates.items():
            for box, prob in zip(pred['boxes'], pred['probs']):
                x1, y1, x2, y2 = map(int, box)
                box_area = (x2 - x1) * (y2 - y1)
                # if box_area / img_area >= 0.90:
                #     continue
                pcd = self._bbox_to_pcd(img, depth_m, box, intrinsics, obs_pose)
                if pcd is not None:
                    pcds.append(pcd)
                    bboxes_3d.append(pcd.get_axis_aligned_bounding_box())
                    labels_3d.append(label)
                    probs_3d.append(prob)
        pcd_ms = (time.time() - t1) * 1000

        if pcds:
            t2 = time.time()
            self._update_detections(pcds, bboxes_3d, labels_3d, probs_3d)
            update_ms = (time.time() - t2) * 1000
            total_tracked = sum(len(v) for v in self.detections.values())
            # self.get_logger().info(
            #     f'[process_queue][success]YOLO {yolo_ms:.0f}ms | pcd {pcd_ms:.0f}ms | update {update_ms:.0f}ms '
            #     f'({len(pcds)} PCDs) | tracked={total_tracked}'
            # )
        else:
            # self.get_logger().info(f'[process_queue][failed]YOLO {yolo_ms:.0f}ms | no detections')
            pass
        self.last_header = rgb_msg.header

    def _bbox_to_pcd(self, img, depth_m, box, intrinsics, obs_pose):
        """Back-project the depth pixels inside a 2D bounding box to a 3D point cloud."""
        x1, y1, x2, y2 = map(int, box)
        x1 = max(0, x1);  y1 = max(0, y1)
        x2 = min(depth_m.shape[1], x2);  y2 = min(depth_m.shape[0], y2)
        if x2 <= x1 or y2 <= y1:
            return None

        roi_depth = depth_m[y1:y2, x1:x2]
        roi_rgb   = img[y1:y2, x1:x2]

        fx, fy = intrinsics['fx'], intrinsics['fy']
        cx, cy = intrinsics['cx'], intrinsics['cy']

        us = np.arange(x1, x2, dtype=np.float32)
        vs = np.arange(y1, y2, dtype=np.float32)
        grid_u, grid_v = np.meshgrid(us, vs)

        z = roi_depth.flatten()
        valid = (z > CAMERA_MIN_RANGE_M) & (z < CAMERA_MAX_RANGE_M)
        z = z[valid]
        x = (grid_u.flatten()[valid] - cx) * z / fx
        y = (grid_v.flatten()[valid] - cy) * z / fy
        pts    = np.stack([x, y, z], axis=1).astype(np.float32)
        colors = roi_rgb.reshape(-1, 3)[valid].astype(np.float32)
        colors = colors[:, ::-1] / 255.0  # BGR → RGB, normalise to [0,1]

        if pts.shape[0] == 0:
            return None
        pcd = o3tg.PointCloud()
        pcd.point["positions"] = o3c.Tensor(pts,    o3c.float32)
        pcd.point["colors"]    = o3c.Tensor(np.ascontiguousarray(colors), o3c.float32)
        if DO_VOXEL:
            pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE_M)

        if RADIUS_OUTLIER_REMOVAL:
            pcd, _ = pcd.remove_radius_outliers(
                nb_points=RADIUS_OUTLIER_NB_POINTS,
                search_radius=RADIUS_OUTLIER_RADIUS,
            )
        if STATISTICAL_OUTLIER_REMOVAL:
            pcd, _ = pcd.remove_statistical_outliers(
                nb_neighbors=STATISTICAL_OUTLIER_NB_NEIGHBORS,
                std_ratio=STATISTICAL_OUTLIER_STD_RATIO,
            )

        if "positions" not in pcd.point or pcd.point["positions"].shape[0] < PCD_MIN_POINTS:
            return None

        pcd = pcd.transform(obs_pose)
        return pcd

    def _update_detections(self, pcds, bboxes_3d, labels, probs):
        """
        For each new detection, check if its bounding box overlaps an existing one
        of the same label by at least OVERLAP_THRESHOLD. If so, merge the PCDs.
        Otherwise add as new.
        """
        for pcd, bbox, label, prob in zip(pcds, bboxes_3d, labels, probs):
            matched = False
            for det in self.detections.get(label, []):
                if self._bbox_overlap_ratio(bbox, det['bbox']) >= OVERLAP_THRESHOLD:
                    merged = self._merge_pcds(det['pcd'], pcd)
                    det['pcd']  = merged
                    det['bbox'] = merged.get_axis_aligned_bounding_box()
                    det['prob'] = max(det['prob'], prob)
                    matched = True
                    break
            if not matched:
                self.detections.setdefault(label, []).append({
                    'pcd':  pcd,
                    'bbox': bbox,
                    'prob': prob,
                })

    def _bbox_overlap_ratio(self, bbox1, bbox2):
        """Intersection volume / min(vol1, vol2) for two AxisAlignedBoundingBoxes."""
        mn1 = bbox1.min_bound.numpy();  mx1 = bbox1.max_bound.numpy()
        mn2 = bbox2.min_bound.numpy();  mx2 = bbox2.max_bound.numpy()

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
        """Stack two tensor PointClouds, voxel-downsample, and re-run outlier removal."""
        pts = np.vstack([pcd1.point["positions"].numpy(), pcd2.point["positions"].numpy()])
        cls = np.vstack([pcd1.point["colors"].numpy(),    pcd2.point["colors"].numpy()])
        merged = o3tg.PointCloud()
        merged.point["positions"] = o3c.Tensor(pts, o3c.float32)
        merged.point["colors"]    = o3c.Tensor(cls, o3c.float32)
        if DO_VOXEL:
            merged = merged.voxel_down_sample(voxel_size=VOXEL_SIZE_M)
        return merged

    def publish_detections(self):
        """Publish MarkerArray and merged PointCloud2 from all accumulated detections."""
        now = time.time()
        if now - self._last_publish_time < self._publish_interval:
            return
        self._last_publish_time = now
        # self.get_logger().info(f'Publishing {sum(len(v) for v in self.detections.values())} detections...')

        if self.last_annotated is not None:
            self.annotated_pub.publish(self.last_annotated)

        if not self.detections or self.last_header is None:
            return

        lines = []
        for label, dets in self.detections.items():
            for i, det in enumerate(dets):
                mn = det['bbox'].min_bound.numpy()
                mx = det['bbox'].max_bound.numpy()
                center = (mn + mx) / 2.0
                n_pts  = det['pcd'].point["positions"].shape[0]
                lines.append(
                    f'  [{label}#{i}] center=({center[0]:.3f},{center[1]:.3f},{center[2]:.3f}) '
                    f'pts={n_pts} prob={det["prob"]:.2f}'
                )
        # self.get_logger().info('publish_detections:\n' + '\n'.join(lines))

        # MarkerArray — one CUBE per detection, namespaced by label
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = BASE_FRAME
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        for label_idx, (label, dets) in enumerate(self.detections.items()):
            for i, det in enumerate(dets):
                mn = det['bbox'].min_bound.numpy()
                mx = det['bbox'].max_bound.numpy()
                center = (mn + mx) / 2.0
                size   = mx - mn

                m = Marker()
                m.header          = self.last_header
                m.header.frame_id = BASE_FRAME
                m.ns              = label
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
                m.color   = MARKER_COLORS[label_idx % len(MARKER_COLORS)]
                markers.markers.append(m)
        self.markers_pub.publish(markers)
        # self.get_logger().info(f'Published MarkerArray with {len(markers.markers)-1} markers')

        # PointCloud2 — all detection PCDs merged into one message
        all_dets = [det for dets in self.detections.values() for det in dets]
        all_pts = np.vstack([d['pcd'].point["positions"].numpy() for d in all_dets])
        all_cls = np.vstack([d['pcd'].point["colors"].numpy()    for d in all_dets])

        combined = o3tg.PointCloud()
        combined.point["positions"] = o3c.Tensor(all_pts, o3c.float32)
        combined.point["colors"]    = o3c.Tensor(np.ascontiguousarray(all_cls), o3c.float32)
        combined = combined.voxel_down_sample(voxel_size=VOXEL_SIZE_M)
        all_pts = combined.point["positions"].numpy()
        all_cls = combined.point["colors"].numpy()

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
        # self.get_logger().info(f'Published PointCloud2 with {len(all_pts)} points')

    def _clear_detections_cb(self, request, response):
        with self.lock:
            self.detections.clear()
        self.get_logger().info('Detections cleared.')
        response.success = True
        response.message = 'Detections cleared.'
        return response

    def _get_detections_json_cb(self, request, response):
        try:
            result = {}
            with self.lock:
                for label, dets in self.detections.items():
                    for i, det in enumerate(dets):
                        mn = det['bbox'].min_bound.numpy()
                        mx = det['bbox'].max_bound.numpy()
                        centroid = ((mn + mx) / 2.0).tolist()
                        result[f'{label}_{i}'] = centroid
            response.success = True
            response.message = json.dumps(result)
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'get_detections_json failed: {e}')
        return response

    def _save_detections_cb(self, request, response):
        path = self.get_parameter('detections_save_path').get_parameter_value().string_value
        try:
            save_data = {}
            with self.lock:
                for label, dets in self.detections.items():
                    save_data[label] = [
                        {
                            'positions': det['pcd'].point["positions"].numpy(),
                            'colors':    det['pcd'].point["colors"].numpy(),
                            'prob':      det['prob'],
                        }
                        for det in dets
                    ]
            with open(path, 'wb') as f:
                pickle.dump(save_data, f)
            n = sum(len(v) for v in save_data.values())
            response.success = True
            response.message = f'Saved {n} detections to {path}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'save_object_detection failed: {e}')
        return response

    def _load_detections(self, path):
        with open(path, 'rb') as f:
            save_data = pickle.load(f)
        new_detections = {}
        for label, dets in save_data.items():
            new_detections[label] = []
            for det in dets:
                pcd = o3tg.PointCloud()
                pcd.point["positions"] = o3c.Tensor(det['positions'].astype(np.float32), o3c.float32)
                pcd.point["colors"]    = o3c.Tensor(det['colors'].astype(np.float32),    o3c.float32)
                new_detections[label].append({
                    'pcd':  pcd,
                    'bbox': pcd.get_axis_aligned_bounding_box(),
                    'prob': det['prob'],
                })
        with self.lock:
            self.detections = new_detections
        n = sum(len(v) for v in new_detections.values())
        self.get_logger().info(f'Loaded {n} detections from {path}')

    def _load_detections_cb(self, request, response):
        path = self.get_parameter('detections_save_path').get_parameter_value().string_value
        try:
            self._load_detections(path)
            response.success = True
            response.message = f'Loaded detections from {path}'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'load_object_detection failed: {e}')
        return response

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
