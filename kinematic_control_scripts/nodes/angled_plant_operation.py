#!/usr/bin/env python3

import os
import time
import json
import warnings
import numpy as np
import cv2
import rospy
import pyrealsense2
import tf2_ros
import tf2_geometry_msgs
import matplotlib.pyplot as plt
from copy import deepcopy
from math import pi
from geometry_msgs.msg import Pose, PointStamped, PoseArray, Point
from sensor_msgs.msg import Image, CameraInfo
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
from kg_robot import kg_robot as kgr

warnings.filterwarnings("ignore")
setup_logger()

# ROS Topics
BBOX_TO_WORLD_COORDINATES = '/bbox_pixel_to_world_coordinates'
TCP_WORLD_COORDINATES = '/tcp_world_coordinates'
OVERHEAD_CAMERA_COLOR_TOPIC_NAME = '/camera1/color/image_raw'

class PlantOperation:
    def __init__(self):
        self.home_pose = Pose()
        self.start_plant_pose = Pose()
        self.color_image = None
        self.bbox_location = None
        self.acc_val = 0.5
        self.vel_val = 0.5
        self.tcp_location = None
        self.record_trajectory = False
        self.trajectory_points = []
        self.trajectory_start_time = None
        
        print("------------Configuring Spudnik, the Potato Planter -------------\r\n")
        self.spudnik = kgr.kg_robot(port=30010, db_host="192.168.2.10")
        self.digital_pin = 0
        self.valve_off = 1
        self.valve_on = 0
        self.bridge = CvBridge()
        
        rospy.Subscriber(BBOX_TO_WORLD_COORDINATES, Pose, self.pose_callback)
        rospy.Subscriber(OVERHEAD_CAMERA_COLOR_TOPIC_NAME, Image, self.camera_color_callback)
        rospy.Subscriber(TCP_WORLD_COORDINATES, Point, self.tcp_callback)
        
        self.setup_starting_poses()
    
    def pose_callback(self, data):
        self.bbox_location = data
    
    def camera_color_callback(self, data):
        self.color_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    
    def tcp_callback(self, data):
        self.tcp_location = data
        if self.record_trajectory:
            current_time = time.time() - self.trajectory_start_time
            self.trajectory_points.append([data.x, data.y, data.z, current_time])
    
    def reset_trajectory_tracking(self):
        self.trajectory_start_time = None
        self.trajectory_points = []
        self.record_trajectory = False
    
    def start_trajectory_tracking(self):
        self.trajectory_start_time = time.time()
        self.record_trajectory = True
    
    def setup_starting_poses(self):
        self.home_pose.position.x, self.home_pose.position.y, self.home_pose.position.z = -0.15, -0.90, 0.453
        self.home_pose.orientation.x, self.home_pose.orientation.y, self.home_pose.orientation.z, self.home_pose.orientation.w = 0.0132, -0.0318, -0.9994, 0.0078
        
        self.start_plant_pose.position.x, self.start_plant_pose.position.y, self.start_plant_pose.position.z = 0.4, 0.3, 0.6
        self.start_plant_pose.orientation.x, self.start_plant_pose.orientation.y, self.start_plant_pose.orientation.z, self.start_plant_pose.orientation.w = 0.0132, -0.0318, -0.9994, 0.0078
    
    def main(self):
        self.spudnik.set_digital_out(self.digital_pin, self.valve_off)
        home_pose_robot = self.spudnik.ros_to_robot_pose_transform_no_rotation(self.home_pose)
        self.spudnik.movel(home_pose_robot, acc=self.acc_val, vel=self.vel_val)
        
        while not rospy.is_shutdown():
            self.bbox_location = None
            home_pose_robot = self.spudnik.ros_to_robot_pose_transform_no_rotation(self.home_pose)
            self.spudnik.movel(home_pose_robot, acc=self.acc_val, vel=self.vel_val)
            rospy.sleep(1)
            
            if self.bbox_location is None:
                print("No more potatoes detected, finishing.")
                self.spudnik.movel(home_pose_robot, acc=self.acc_val, vel=self.vel_val)
                break
            
            bbox = deepcopy(self.bbox_location)
            pose_above = deepcopy(bbox)
            pose_above.position.z += 0.1
            pose_above.position.x -= 0.15
            pose_above_robot = self.spudnik.ros_to_robot_pose_transform_at_angle(pose_above)
            self.spudnik.movel(pose_above_robot, acc=self.acc_val, vel=self.vel_val)
            
            pick_pose = deepcopy(pose_above)
            pick_pose.position.z -= 0.075
            pick_pose.position.x += 0.12
            pick_pose_robot = self.spudnik.ros_to_robot_pose_transform_at_angle(pick_pose)
            self.spudnik.movel(pick_pose_robot, acc=self.acc_val, vel=self.vel_val)
            self.spudnik.set_digital_out(self.digital_pin, self.valve_on)
            rospy.sleep(1)
            
            self.spudnik.movel(pose_above_robot, acc=self.acc_val, vel=self.vel_val)
            self.spudnik.movel(home_pose_robot, acc=self.acc_val, vel=self.vel_val)
        
        print("Goodbye")
        self.spudnik.close()

if __name__ == '__main__':
    rospy.init_node('plant_operation', anonymous=True)
    plant = PlantOperation()
    plant.main()
