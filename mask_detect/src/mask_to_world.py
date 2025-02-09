#!/usr/bin/env python3

import os
import time
import json
import warnings
import numpy as np
import cv2
import torch
import rospy
import pyrealsense2
import tf2_ros
import tf2_geometry_msgs
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PointStamped
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge
from sklearn.decomposition import PCA
from detectron2.utils.logger import setup_logger
from detectron2.model_zoo import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.structures import BoxMode

warnings.filterwarnings("ignore")
setup_logger()

# ROS Topics
OVERHEAD_CAMERA_COLOR_TOPIC_NAME = '/camera1/color/image_raw'
OVERHEAD_CAMERA_DEPTH_TOPIC_NAME = '/camera1/depth/image_rect_raw'
OVERHEAD_CAMERA_INFO_TOPIC_NAME = '/camera1/color/camera_info'
BBOX_TO_WORLD_COORDINATES = '/bbox_pixel_to_world_coordinates'

class PotatoLocaliser:
    def __init__(self):
        self.camera_model = None
        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.transform = None
        self.bridge = CvBridge()
        self.predictor = None
        self.cfg = None
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        rospy.Subscriber(OVERHEAD_CAMERA_INFO_TOPIC_NAME, CameraInfo, self.camera_info_callback)
        rospy.Subscriber(OVERHEAD_CAMERA_DEPTH_TOPIC_NAME, Image, self.camera_depth_callback)
        rospy.Subscriber(OVERHEAD_CAMERA_COLOR_TOPIC_NAME, Image, self.camera_color_callback)
        self.coordinates_publisher = rospy.Publisher(BBOX_TO_WORLD_COORDINATES, Pose, queue_size=10)

        self.get_intrinsics()

    def get_intrinsics(self):
        rospy.sleep(3)
        self.intrinsics = pyrealsense2.intrinsics()
        self.intrinsics.width = self.camera_model.width
        self.intrinsics.height = self.camera_model.height
        self.intrinsics.ppx = self.camera_model.K[2]
        self.intrinsics.ppy = self.camera_model.K[5]
        self.intrinsics.fx = self.camera_model.K[0]
        self.intrinsics.fy = self.camera_model.K[4]
        self.intrinsics.model = pyrealsense2.distortion.none  
        self.intrinsics.coeffs = [i for i in self.camera_model.D]

    def get_world_to_camera_transform(self):
        while not rospy.is_shutdown():
            try:
                self.transform = self.tfBuffer.lookup_transform('base_link', 'overhead_camera', rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

    def camera_info_callback(self, data):
        self.camera_model = data

    def camera_color_callback(self, data):
        self.color_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def camera_depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def setup_model(self):
        for d in ["train", "val"]:
            DatasetCatalog.register(f"potatoes_{d}", lambda d=d: self.get_potatoes_dicts(f"potatoes/{d}"))
            MetadataCatalog.get(f"potatoes_{d}").set(thing_classes=["Jersey Royal", "Handle Bar"])

        self.potatoes_metadata = MetadataCatalog.get("potatoes_train")
        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.85
        self.cfg.MODEL.WEIGHTS = os.path.expanduser("~/pp_ws/src/mask_detect/src/output/model_final_bar.pth")
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 2
        self.predictor = DefaultPredictor(self.cfg)

    def convert_depth_to_phys_coord_using_realsense(self, x, y, depth):  
        result = pyrealsense2.rs2_deproject_pixel_to_point(self.intrinsics, [x, y], depth)  
        return result[2], -result[0], -result[1]

    def main(self):
        self.setup_model()
        self.get_world_to_camera_transform()
        
        while not rospy.is_shutdown():
            self.get_world_to_camera_transform()
            outputs = self.predictor(self.color_image[..., ::-1])
            v = Visualizer(self.color_image[:, :, ::-1], metadata=self.potatoes_metadata, scale=1.0)
            out_image = v.draw_instance_predictions(outputs["instances"].to("cpu")).get_image()[..., ::-1]
            
            try:
                box = outputs["instances"].pred_boxes.tensor.cpu().numpy()
                x_centre = int((box[0][0] + box[0][2]) / 2)
                y_centre = int((box[0][1] + box[0][3]) / 2)
                depth = self.depth_image[y_centre, x_centre]
                xyz_camera_frame = self.convert_depth_to_phys_coord_using_realsense(x_centre, y_centre, depth)
                
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = xyz_camera_frame
                self.coordinates_publisher.publish(pose)
            except Exception as e:
                print(f"Error: {e}")

            cv2.imshow("Detections", out_image)
            cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('pixel_to_robot_coordinates', anonymous=True)
    potato = PotatoLocaliser()
    potato.main()
