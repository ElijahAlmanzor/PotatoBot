#!/usr/bin/env python3

import os
import sys
import time
import json
import warnings
import numpy as np
import cv2
import random
import torch
import torchvision
import pyrealsense2
import rospy
import tf2_ros
import tf2_geometry_msgs
import matplotlib.pyplot as plt
from PIL import Image as Im
from PIL import ImageEnhance
from sklearn.decomposition import PCA
from sklearn import datasets, linear_model
from sklearn.metrics import mean_squared_error, r2_score
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PointStamped, PoseArray
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge

# Detectron2 Imports
import detectron2
from detectron2.utils.logger import setup_logger
from detectron2.model_zoo import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.structures import BoxMode
from detectron2.evaluation import COCOEvaluator, inference_on_dataset
from detectron2.data import build_detection_test_loader

# Ignore warnings (for deprecated functions)
warnings.filterwarnings("ignore")

class GroundTruthExtract:
    """
    Class for analyzing real vs. robotic potato planting using object detection.
    """
    def __init__(self):
        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.transform = None
        self.bridge = CvBridge()
        self.predictor = None
        self.cfg = None

    def get_potatoes_dicts(self, img_dir):
        """Loads the JSON dataset annotations and converts them into Detectron2 format."""
        json_file = os.path.join(img_dir, "potato_d2_dataset.json")
        with open(json_file) as f:
            imgs_anns = json.load(f)
        
        dataset_dicts = []
        for idx, v in enumerate(imgs_anns.values()):
            filename = os.path.join(img_dir, v["filename"])
            height, width = cv2.imread(filename).shape[:2]
            record = {
                "file_name": filename,
                "image_id": idx,
                "height": height,
                "width": width,
                "annotations": [
                    {
                        "bbox": [np.min(anno["all_points_x"]), np.min(anno["all_points_y"]),
                                  np.max(anno["all_points_x"]), np.max(anno["all_points_y"])],
                        "bbox_mode": BoxMode.XYXY_ABS,
                        "segmentation": [[(x + 0.5, y + 0.5) for x, y in zip(anno["all_points_x"], anno["all_points_y"])][i] for i in range(len(anno["all_points_x"]))],
                        "category_id": 0,
                    }
                    for anno in v["regions"]
                ],
            }
            dataset_dicts.append(record)
        return dataset_dicts

    def setup_model(self, threshold=0.5):
        """Initializes and configures the Detectron2 model."""
        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = threshold
        self.cfg.MODEL.WEIGHTS = os.path.expanduser("~/pp_ws/src/mask_detect/src/output/model_final.pth")
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
        self.predictor = DefaultPredictor(self.cfg)
        self.potatoes_metadata = MetadataCatalog.get("potatoes_train")

    def detect_potatoes(self, image_path):
        """Runs the trained model on an image and visualizes detections."""
        color_image = cv2.imread(image_path, cv2.COLOR_BGR2RGB)
        outputs = self.predictor(color_image[..., ::-1])
        visualizer = Visualizer(color_image[:, :, ::-1], metadata=self.potatoes_metadata, scale=1.0)
        out_image = visualizer.draw_instance_predictions(outputs["instances"].to("cpu")).get_image()[..., ::-1]
        
        cv2.imshow("Detected Potatoes", out_image)
        cv2.waitKey(0)
        return outputs

    def compute_statistics(self, outputs):
        """Computes statistics like gaps, straightness, and angles of detected potatoes."""
        boxes = outputs["instances"].pred_boxes.tensor.cpu().numpy()
        boxes = sorted(boxes, key=lambda x: x[0])  # Sort by leftmost x-coordinate
        gaps = np.diff([box[2] for box in boxes])
        avg_gap = np.mean(gaps) if gaps.size else 0
        std_gap = np.std(gaps) if gaps.size else 0
        print(f"Avg gap: {avg_gap}, Std gap: {std_gap}")
        return avg_gap, std_gap

    def analyze_images(self, image_dir):
        """Runs analysis on multiple images."""
        image_files = [os.path.join(image_dir, f) for f in os.listdir(image_dir) if f.endswith('.png')]
        for image_path in image_files:
            outputs = self.detect_potatoes(image_path)
            self.compute_statistics(outputs)

if __name__ == '__main__':
    potato_detector = GroundTruthExtract()
    potato_detector.setup_model(threshold=0.95)
    potato_detector.analyze_images(os.path.expanduser('~/pp_ws/src/mask_detect/src/ground_truths'))
