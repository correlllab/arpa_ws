import torch
import cv2
import shutil
from PIL import Image

import json
import os

import sys


from ultralytics import YOLOWorld
import numpy as np


def assert_candidates2d(candidates_2d):
    """
    Asserts that the candidates_2d dictionary is in the correct format.
    """
    for query_object, prediction in candidates_2d.items():
        assert isinstance(query_object, str), f"candidates_2d key is not a string: {query_object}"
        assert "boxes" in prediction, f"candidates_2d for {query_object} does not contain 'boxes'"
        assert "probs" in prediction, f"candidates_2d for {query_object} does not contain 'probs'"
        assert len(prediction["boxes"]) == len(prediction["probs"]), f"candidates_2d for {query_object} has different number of boxes and probs"
        for bbox in prediction["boxes"]:
            assert len(bbox) == 4, f"candidates_2d for {query_object} contains a box that does not have 4 elements: {bbox}"
            x1, y1, x2, y2 = bbox
            assert x1 < x2, f"candidates_2d for {query_object} contains a box with x1 >= x2: {bbox}"
            assert y1 < y2, f"candidates_2d for {query_object} contains a box with y1 >= y2: {bbox}"
        for prob in prediction["probs"]:
            assert 0.0 <= prob <= 1.0, f"candidates_2d for {query_object} contains a prob that is not between 0 and 1: {prob}"

class YOLO_WORLD:
    def __init__(self, weight_file_path=None):
        """
        Initializes the YOLO World model.
        """
        if weight_file_path is not None:
            assert os.path.exists(weight_file_path), f"Weight file {weight_file_path} does not exist"
            ckpt = torch.load(weight_file_path, map_location='cpu', weights_only=False)
            if isinstance(ckpt, dict) and 'state_dict' in ckpt:
                self.model = YOLOWorld('yolov8x-worldv2.pt')
                self.model.model.load_state_dict(ckpt['state_dict'], strict=False)
            else:
                self.model = YOLOWorld(weight_file_path)
        else:
            self.model = YOLOWorld('yolov8x-worldv2.pt')
        print(f"[YOLO_WORLD init] Successfully initialized with {weight_file_path=}")
        
        device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.model.to(device)

        self.count = 0
        

    def predict(self, img, queries, debug):
        """
        Parameters:
        - img: image to produce bounding boxes in
        - queries: list of strings whose bounding boxes we want
        - debug: if True, prints debug information
        Returns:
        - candidates_2d: dict mapping each query to {"boxes": [...], "probs": [...]}
        """
        self.model.set_classes(queries)
        with torch.no_grad():
            results = self.model.predict(img, show=False, verbose=debug, conf=0.01, nms=True, iou=0.01)[0]
        if debug:
            print(f"[YOLO_WORLD predict]{dir(results.boxes)=}")

        boxes   = results.boxes.xyxy            # (N, 4)
        probs   = results.boxes.conf            # (N,)
        cls_ids = results.boxes.cls.long()      # (N,)


        candidates_2d = {}
        for idx, query in enumerate(queries):
            mask = cls_ids == idx

            selected_boxes = boxes[mask].tolist()  # list of [x1,y1,x2,y2]
            # apply mask after the clamp
            selected_probs = probs[mask].tolist() if isinstance(probs, torch.Tensor) \
                             else [max(float(p), self.vlm_tpr) for p in probs][mask]

            candidates_2d[query] = {
                "boxes": selected_boxes,
                "probs": selected_probs
            }
            if debug:
                print(f"[YOLO_WORLD predict] {query=} probs:{len(selected_probs)=} boxes:{len(selected_boxes)=}")
        assert_candidates2d(candidates_2d)
        total = sum(len(v['boxes']) for v in candidates_2d.values())
        hits  = {q: len(v['boxes']) for q, v in candidates_2d.items() if v['boxes']}
        print(f"[YOLO] {total} detections: {hits}" if total else "[YOLO] no detections")
        return candidates_2d