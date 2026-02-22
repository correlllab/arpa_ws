import time
import torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
import warnings
import numpy as np
import open3d as o3d
import open3d.core as o3c
import open3d.t.geometry as o3tg
import matplotlib
matplotlib.use("Agg")  # non-GUI backend for image writing
import matplotlib.pyplot as plt
import numpy as np
import json
import shutil
import os

import sys

weight_path = os.path.join(os.path.dirname(__file__), "sam2.1_hiera_large.pt")
config_path = "configs/sam2.1/sam2.1_hiera_l.yaml"

min_3d_points = 10
camera_min_range_m = 0.15
camera_max_range_m = 1.5
radius_outlier_removal = False
radius_nb_points = 16
radius_radius = 0.01
statistical_outlier_removal = False
statistical_nb_neighbors = 16
statistical_std_ratio = 2.0
voxel_size_m = 0.005

def assert_well_formed(pcds, bounding_boxes_3d, masked_rgbs, masked_depths, probs):
    lengths = [len(pcds), len(bounding_boxes_3d), len(masked_rgbs), len(masked_depths), len(probs)]
    if not all(l == lengths[0] for l in lengths):
        raise ValueError(f"Lists are not well formed: {lengths=}")
    assert isinstance(pcds, list), f"pcds is not a list: {type(pcds)}"
    assert isinstance(bounding_boxes_3d, list), f"bounding_boxes_3d is not a list: {type(bounding_boxes_3d)}"
    assert isinstance(masked_rgbs, list), f"masked_rgbs is not a list: {type(masked_rgbs)}"
    assert isinstance(masked_depths, list), f"masked_depths is not a list: {type(masked_depths)}"
    assert isinstance(probs, list), f"probs is not a list: {type(probs)}"
    for i in range(lengths[0]):
        pcd = pcds[i]
        bbox = bounding_boxes_3d[i]
        masked_rgb = masked_rgbs[i]
        masked_depth = masked_depths[i]
        prob = probs[i]
        assert isinstance(pcd, o3tg.PointCloud), f"pcd is not a PointCloud: {type(pcd)}"
        assert prob >= 0.0 and prob <= 1.0, f"prob is not in [0, 1]: {prob}"
        assert isinstance(bbox, o3tg.AxisAlignedBoundingBox), f"bbox is not an AxisAlignedBoundingBox: {type(bbox)}"
        assert isinstance(masked_rgb, np.ndarray), f"masked_rgb is not a numpy array: {type(masked_rgb)}"
        assert isinstance(masked_depth, np.ndarray), f"masked_depth is not a numpy array: {type(masked_depth)}"
        assert masked_rgb.ndim == 3 and masked_rgb.shape[2] == 3, f"masked_rgb is not HxWx3: {masked_rgb.shape}"


def get_points_and_colors(depths, rgbs, fx, fy, cx, cy):
    """
    Back-project a batch of depth and RGB images to 3D point clouds.

    Args:
        depths: Tensor of shape (B, H, W) representing depth in meters.
        rgbs: Tensor of shape (B, H, W, 3) representing RGB colors, range [0, 1] or [0, 255].
        fx, fy, cx, cy: camera intrinsics.

    Returns:
        points: Tensor of shape (B, H*W, 3) representing 3D points.
        colors: Tensor of shape (B, H*W, 3) representing RGB colors for each point.
    """
    B, H, W = depths.shape
    device = depths.device

    # Create meshgrid of pixel coordinates
    u = torch.arange(W, device=device)
    v = torch.arange(H, device=device)
    grid_v, grid_u = torch.meshgrid(v, u, indexing='ij')  # (H, W)

    # Flatten pixel coordinates
    grid_u_flat = grid_u.reshape(-1)  # (H*W,)
    grid_v_flat = grid_v.reshape(-1)  # (H*W,)

    # Flatten depth and color
    z = depths.reshape(B, -1)  # (B, H*W)
    colors = rgbs.reshape(B, -1, 3)  # (B, H*W, 3)

    # Back-project to camera coordinates
    x = (grid_u_flat[None, :] - cx) * z / fx  # (B, H*W)
    y = (grid_v_flat[None, :] - cy) * z / fy  # (B, H*W)

    # Stack into point sets
    points = torch.stack((x, y, z), dim=-1)  # (B, H*W, 3)

    return points, colors



#Class to use sam2
class sam2:
    def __init__(self):
        """
        Initializes the SAM2 model and processor.
        """
        
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.o3dDevice = o3c.Device("CPU:0")
        assert os.path.exists(weight_path), f"[SAM2_PC init] Weights at {weight_path} not found"
        sam_model = build_sam2(config_path, weight_path, device=self.device)
        self.sam_predictor = SAM2ImagePredictor(sam_model)
        self.fig_dir = os.path.join(os.path.dirname(__file__), "sam2_debug_figs")
        os.makedirs(self.fig_dir, exist_ok=True)
        print(f"[SAM2_PC init] Successfully SAM2 inilitailzied using device {self.device}, Open3D device {self.o3dDevice}")
        self.count = 0

    def get_masks(self, rgb_img, depth_img, bbox, debug):
        """
        Get masked rgb and depth from SAM2 for the given bounding boxes.
        Parameters:
        - rgb_img: RGB image
        - depth_img: Depth image
        - bbox: Bounding boxes
        """
        #Run sam2 on all the boxes
        if debug:
            print(f"[SAM2_PC get_masks] rgb_img.shape = {rgb_img.shape}")
            print(f"[SAM2_PC get_masks] depth_img.shape = {depth_img.shape}")
            print(f"[SAM2_PC get_masks] bbox = {bbox}, type = {type(bbox)}, np.array(bbox).shape = {np.array(bbox).shape}")
        if len(bbox) == 0:
            if debug:
                print("[SAM2_PC get_masks] no boxes to process, returning empty masks")
            return  None, None
        self.sam_predictor.set_image(rgb_img.copy())
        sam_mask = None
        sam_scores = None
        sam_logits = None
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=UserWarning)
            original_sam_mask, sam_scores, sam_logits = self.sam_predictor.predict(box=bbox)
        if debug:
            print(f"[SAM2_PC get_masks] original_sam_mask.shape = {original_sam_mask.shape}")
            print(f"[SAM2_PC get_masks] sam_scores.shape = {sam_scores.shape}")
            print(f"[SAM2_PC get_masks] sam_logits.shape = {sam_logits.shape}")
        if original_sam_mask.ndim == 3:
            # single mask → add batch axis
            original_sam_mask = original_sam_mask[np.newaxis, ...]
        sam_mask = np.any(original_sam_mask, axis=1)
        if debug:
            print(f"[SAM2_PC get_masks] {sam_mask.shape=}")


        #Apply mask to the depth and rgb images
        #print(f"{original_sam_mask.shape=}, {sam_mask.shape=}, {rgb_img.shape=}, {depth_img.shape=}")
        masked_depth = depth_img[None, ...] * sam_mask
        masked_rgb = rgb_img[None, ...] * sam_mask[..., None]
        return masked_depth, masked_rgb

    def get_pcd(self, pts, cls, transformation_matrix, debug):
        # build Open3D PointCloud
        if debug:
            print(f"\n\n[SAM2_PC get_pcd_bbox] {len(pts)=} {len(cls)=}")
        pcd = o3tg.PointCloud(self.o3dDevice)
        pcd.point["positions"] = o3c.Tensor(pts.numpy().astype(np.float32), o3c.float32, self.o3dDevice)
        pcd.point["colors"] = o3c.Tensor((cls.numpy()/255.0).astype(np.float32), o3c.float32, self.o3dDevice)
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size_m)
        # Apply statistical outlier removal to denoise the point cloud
        if statistical_outlier_removal:
            pcd, ind = pcd.remove_statistical_outliers(nb_neighbors=statistical_nb_neighbors, std_ratio=statistical_std_ratio)
        if radius_outlier_removal:
            pcd, ind = pcd.remove_radius_outliers(nb_points=radius_nb_points, search_radius=radius_radius)
        pcd = pcd.transform(transformation_matrix)
        
        return pcd

    def save_masks(self, rgb_masks, query_str):
        side_length = np.ceil(np.sqrt(len(rgb_masks))).astype(int)
        side_length = max(side_length, 2)
        fig, axes = plt.subplots(side_length, side_length, figsize=(15, 15))
        axes = axes.flatten()
        for i, (ax, mask) in enumerate(zip(axes, rgb_masks)):
            ax.imshow(mask)
            ax.axis('off')
        fig.suptitle(f"RGB Masks for {query_str}")
        plt.tight_layout()
        plt.savefig(os.path.join(self.fig_dir, f"{self.count:04d}_{query_str.replace(' ', '_')}_rgb_masks.png"))
        plt.close(fig)
        self.count += 1

    def predict(self, rgb_img, depth_img, bbox, probs, intrinsics, obs_pose, debug, query_str=""):
        """
        Predicts 3D point clouds from RGB and depth images and bounding boxes using SAM2.
        Cleans up the point clouds and applies NMS.
        Parameters:
        - rgb_img: RGB image
        - depth_img: Depth image
        - bbox: Bounding boxes
        - intrinsics: Camera intrinsics
        - debug: If True, prints debug information
        Returns:
        - pcds: List of reduced point clouds
        - bounding_boxes_3d: List of reduced 3D bounding boxes
        - new_probs: List of reduced scores
        - masked_rgb: List of RGB masks
        - masked_depth: List of Depth Masks
        """
        t0 = time.time()
        print(f"[SAM2] predict called: {len(bbox)} boxes")
        if debug:
            print(f"[SAM2_PC predict] Received {len(bbox)=}, {len(probs)=}")
            print(f"[SAM2_PC predict] query_str = {query_str}")
        masked_depth, masked_rgb = self.get_masks(rgb_img, depth_img, bbox, debug=debug)
        if masked_depth is None or masked_rgb is None:
            return [], [], [], [], []
        if debug:
            print(f"[SAM2_PC predict] masked_depth.shape = {masked_depth.shape}")
            print(f"[SAM2_PC predict] masked_rgb.shape   = {masked_rgb.shape}")
        tensor_depth = torch.from_numpy(masked_depth).to(self.device)
        tensor_rgb = torch.from_numpy(masked_rgb).to(self.device)

        fx = intrinsics["fx"]
        fy = intrinsics["fy"]
        cx = intrinsics["cx"]
        cy = intrinsics["cy"]
        points, colors = get_points_and_colors(tensor_depth, tensor_rgb, fx, fy, cx, cy)
        colors = colors[..., [2, 1, 0]]
        if debug:
            print(f"[SAM2_PC predict] {points.shape=}, {colors.shape=}")

        B, N, _ = points.shape

        pcds = []
        bounding_boxes_3d = []
        new_masked_rgb = []
        new_masked_depth = []
        new_probs = []

        pts_cpu = points.detach().cpu()
        cols_cpu = colors.detach().cpu()
        #for each candiate object get the point cloud unless there are too few points
        transformation_matrix = obs_pose
        for i in range(B):
            pts = pts_cpu[i]
            cls = cols_cpu[i]

            # mask out void points
            depths = pts[:, 2]
            valid = (depths > camera_min_range_m) & (depths < camera_max_range_m)
            if debug:
                print(f"[SAM2_PC predict] {valid.sum()=}")
            if valid.sum() < min_3d_points:
                continue
            pts = pts[valid]
            cls = cls[valid]
            pcd = self.get_pcd(pts, cls, transformation_matrix, debug=debug)
            if pcd.point["positions"].shape[0] < min_3d_points:
                continue
            bbox_3d = pcd.get_axis_aligned_bounding_box()
            if debug:
                print(f"[SAM2_PC get_pcd_bbox] {pcd=}")
                print(f"[SAM2_PC get_pcd_bbox] {bbox_3d=}")

            pcds.append(pcd)
            bounding_boxes_3d.append(bbox_3d)
            new_masked_rgb.append(masked_rgb[i])
            new_masked_depth.append(masked_depth[i])
            new_probs.append(probs[i])
        assert_well_formed(pcds, bounding_boxes_3d, new_masked_rgb, new_masked_depth, new_probs)
        sizes = [pcd.point["positions"].shape[0] for pcd in pcds]
        print(f"[SAM2] {len(bbox)} boxes → {len(pcds)} PCDs | pts={sizes} | {time.time()-t0:.2f}s")
        return pcds, bounding_boxes_3d, new_probs, new_masked_rgb, new_masked_depth
    def __str__(self):
        return f"SAM2: {self.sam_predictor.model.device}"
    def __repr__(self):
        return self.__str__()

        