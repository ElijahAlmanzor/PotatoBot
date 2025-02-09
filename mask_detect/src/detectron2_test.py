#!/usr/bin/env python3

import os
import sys
import time
import json
import warnings
import numpy as np
import cv2
import torch
import torchvision
import rospy
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from detectron2.utils.logger import setup_logger
from detectron2.model_zoo import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.structures import BoxMode

warnings.filterwarnings("ignore")
setup_logger()

OVERHEAD_CAMERA_TOPIC_NAME = '/camera1/color/image_raw'
image = None
bridge = None

def camera_image_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

def get_potatoes_dicts(img_dir):
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

def main():
    global image, bridge
    rospy.init_node('detectron_stream_test', anonymous=True)
    bridge = CvBridge()
    rospy.Subscriber(OVERHEAD_CAMERA_TOPIC_NAME, Image, camera_image_callback)
    rospy.sleep(3)

    for d in ["train", "val"]:
        DatasetCatalog.register("potatoes_" + d, lambda d=d: get_potatoes_dicts("potatoes/" + d))
        MetadataCatalog.get("potatoes_" + d).set(thing_classes=["Potato"])
    
    potatoes_metadata = MetadataCatalog.get("potatoes_train")
    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.98
    cfg.MODEL.WEIGHTS = os.path.expanduser("~/pp_ws/src/mask_detect/src/output/model_final.pth")
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
    predictor = DefaultPredictor(cfg)

    while not rospy.is_shutdown():
        if image is None:
            continue
        outputs = predictor(image[..., ::-1])
        v = Visualizer(image[:, :, ::-1], metadata=potatoes_metadata, scale=1.0)
        out_image = v.draw_instance_predictions(outputs["instances"].to("cpu")).get_image()[..., ::-1]
        
        try:
            mask_instances = np.asarray(outputs["instances"].pred_masks.to("cpu").numpy())
            largest_potato_index = np.argmax([np.count_nonzero(mask) for mask in mask_instances])
            box = outputs["instances"].pred_boxes.tensor.cpu().numpy()[largest_potato_index]
            x_l, y_l, x_r, y_r = map(int, box)
            segmented_image = image[y_l:y_r, x_l:x_r]
            mask = mask_instances[largest_potato_index][y_l:y_r, x_l:x_r] * 255
            disp_mask = mask.astype(np.uint8)
            
            one_indices = np.transpose(np.where(mask == 255))
            y_data, x_data = one_indices[:, 0], one_indices[:, 1]
            data = np.column_stack((x_data, -y_data))
            pca = PCA(n_components=1).fit(data)
            comp = pca.components_[0] * 20
            angle = np.arctan2(comp[1], comp[0]) * 180 / np.pi
            print(f"Angle in degrees: {angle:.2f}")
            
            plt.plot([-comp[0], comp[0]], [-comp[1], comp[1]], linewidth=5, color="red")
            plt.scatter(data[:, 0], data[:, 1])
            plt.show()
            
            cv2.imshow("Segmented Image", segmented_image)
            cv2.imshow("Mask", disp_mask)
        except Exception as e:
            print(f"Error: {e}")
        
        cv2.imshow("Detections", out_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    main()
