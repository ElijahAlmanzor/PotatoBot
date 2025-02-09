#!/usr/bin/env python3

import time
import cv2
import numpy as np
import rospy
import pyrealsense2
import tf2_ros
import tf2_geometry_msgs
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy


import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PointStamped, TransformStamped
from darknet_ros_msgs.msg import BoundingBoxes, LettuceHypothesis, LettuceHypothesisSet
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler, euler_from_quaternion


#For subscribing to the topic 
OVERHEAD_CAMERA_TOPIC_NAME = '/camera1/color/image_raw'
OVERHEAD_CAMERA_RGB_TOPIC_NAME = '/camera1/color/image_raw'
OVERHEAD_CAMERA_DEPTH_TOPIC_NAME = '/camera1/depth/image_rect_raw'
OVERHEAD_CAMERA_INFO_TOPIC_NAME = '/camera1/color/camera_info'

YOLO_BOUNDING_BOXES = '/darknet_ros/bounding_boxes'
BBOX_TO_WORLD_COORDINATES = '/bbox_pixel_to_world_coordinates'



class camera_pose:
    def __init__(self):
        #image subscribers
        self.camera_color_subscriber = rospy.Subscriber(OVERHEAD_CAMERA_RGB_TOPIC_NAME, Image, self.camera_color_callback)
        self.camera_depth_subscriber = rospy.Subscriber(OVERHEAD_CAMERA_DEPTH_TOPIC_NAME, Image, self.camera_depth_callback)
        self.camera_info_subscriber = rospy.Subscriber(OVERHEAD_CAMERA_INFO_TOPIC_NAME, CameraInfo, self.camera_info_callback)
        self.image_points = None
        self.ray_points = None
        self.intrinsics = None
        self.bridge = CvBridge()
        #Variables for storing the rotation and translation matrices
        self.rotation_vector = [0,0,0]
        self.quat_vector = [0,0,0,0]
        self.translation_vector = [0,0,0]
        self.broadcaster =  tf2_ros.StaticTransformBroadcaster()

    def GetIntrinsics(self):
            print("waiting for the camera")
            countdown = 3
            for i in range(countdown):
                rospy.sleep(1)
                print("countdown", countdown-(i))
            #https://medium.com/@yasuhirachiba/converting-2d-image-coordinates-to-3d-coordinates-using-ros-intel-realsense-d435-kinect-88621e8e733a
            self.intrinsics = pyrealsense2.intrinsics()
            self.intrinsics.width = self.camera_model.width
            self.intrinsics.height = self.camera_model.height
            self.intrinsics.ppx = self.camera_model.K[2]
            self.intrinsics.ppy = self.camera_model.K[5]
            self.intrinsics.fx = self.camera_model.K[0]
            self.intrinsics.fy = self.camera_model.K[4]
            #_intrinsics.model = cameraInfo.distortion_model
            self.intrinsics.model  = pyrealsense2.distortion.none  
            self.intrinsics.coeffs = [i for i in self.camera_model.D]


            
            #print(self._intrinsics.width)

    def camera_depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    
    def camera_color_callback(self, data):
        self.color_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

    def camera_info_callback(self, data):
        self.camera_model = data




    def main(self):
        #initialise it once
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "base_link"
        static_transformStamped.child_frame_id = "overhead_camera"
        rospy.sleep(3)
        while not rospy.is_shutdown():
            
            color_image = self.color_image
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
            parameters =  aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            frame_markers = aruco.drawDetectedMarkers(color_image, corners, ids)

            #base frame robot
            '''
            real_points = { '35': [399.23*0.001,104.31*0.001,-1.98*0.001],
                            '15': [ 151.70*0.001,413.83*0.001, 297.5*0.001], #this is the one on the robot end-effector
                            '9': [-287.39*0.001, 106.62*0.001,  -2.10*0.001],
                            '34': [399.80*0.001,6.00*0.001, -2.95*0.001],
                            '10': [-386.23*0.001,-11.51*0.001,-2.56*0.001],
                            #'12': [-85, 310, -1.3],
                            '11': [-389.99*0.001,106.61*0.001,  -1.36*0.001]}
            '''
            #base frame rviz
            real_points = { '35': [104.31*0.001,-399.23*0.001,-1.98*0.001],
                            '15': [ 413.83*0.001,-103.70*0.001, 297.5*0.001], #this is the one on the robot end-effector
                            '9': [ 106.62*0.001,287.39*0.001,  -2.10*0.001],
                            '34': [6.00*0.001,-399.80*0.001, -2.95*0.001],
                            #'10': [-11.51*0.001,386.23*0.001,-2.56*0.001],
                            #'12': [-85, 310, -1.3],
                            '11': [106.61*0.001, 389.99*0.001,  -1.36*0.001]}
            try:
                #print(ids)

                image_points = []
                model_points = []
                for i, id in enumerate(ids):
                    try:
                        c = corners[i][0]
                        model_points.append(real_points[str(id[0])])
                        point = [c[:, 0].mean(), c[:, 1].mean()]
                        #print(id)
                        #print([c[:, 0].mean(), c[:, 1].mean()])
                        image_points.append(point)#
                    except:
                        pass


                image_points = np.asarray(image_points, dtype = "double")
                model_points = np.asarray(model_points, dtype = "double")
                camera_matrix = np.asarray(self.camera_model.K).reshape(3,3)
                dist_coeff = np.asarray([i for i in self.camera_model.D]).reshape(5,1)
                (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeff, flags = 1)
                #(success, rotation_vector, translation_vector, _) = cv2.solvePnPRansac(model_points, image_points, camera_matrix, dist_coeff)#, flags = cv2.SOLVEPNP_P3P)


                self.translation_vector[0] = float(translation_vector[1])
                self.translation_vector[1] = float(translation_vector[0])
                self.translation_vector[2] = float(translation_vector[2])

                static_transformStamped.transform.translation.x = 0.444
                static_transformStamped.transform.translation.y = 0.065
                static_transformStamped.transform.translation.z = 1.111
                #print(self.translation_vector)
                #self.rotation_vector[0] = float(rotation_vector[1])
                #self.rotation_vector[1] = float(rotation_vector[0])
                #self.rotation_vector[2] = float(rotation_vector[2])
                self.quat_vector = quaternion_from_euler(rotation_vector[1],-rotation_vector[0],rotation_vector[2])                
                
                
                
                
                #static_transformStamped.transform.translation.x = self.translation_vector[0]#*0.001
                #static_transformStamped.transform.translation.y = self.translation_vector[1]#*0.001
                #static_transformStamped.transform.translation.z = self.translation_vector[2]#*0.001

                #quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
                static_transformStamped.transform.rotation.x = self.quat_vector[0] 
                static_transformStamped.transform.rotation.y = self.quat_vector[1] 
                static_transformStamped.transform.rotation.z = self.quat_vector[2] 
                static_transformStamped.transform.rotation.w = self.quat_vector[3] 

                #publishes the overhead_camera to the world transformation
                
                
                #print("Rotation Vector (extracted):\n {0}".format(self.rotation_vector))
                #print("Rotation Vector (Quaternion):\n {0}".format(quat))
                #print("Perfect rotation", euler_from_quaternion([0,0.707,0,0.707]))



                #print("Translation Vector:\n {0}".format(self.translation_vector))
                #print(image_points)
                #print(model_points)
            except:
               pass
            
            '''
            self.translation_vector[0] = float(translation_vector[1])
            self.translation_vector[1] = float(translation_vector[0])
            self.translation_vector[2] = float(translation_vector[2])
            '''
            self.broadcaster.sendTransform(static_transformStamped)
            print(static_transformStamped)


            cv2.imshow("Aruco Detect", color_image)
            cv2.waitKey(1)
            #except:
                #print("Some error")
             #   x = 1

   

def main():
    #Taken straight from the vegebot urdf
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link"
    static_transformStamped.child_frame_id = "overhead_camera"

    #static_transformStamped.transform.translation.x = 0.458
    #static_transformStamped.transform.translation.y = 0.020
    #static_transformStamped.transform.translation.z = 1.070

    #As measured from the tool end-effector
    #measured points are 448.75, 63.37, 1111.23
    static_transformStamped.transform.translation.x = 0.451
    static_transformStamped.transform.translation.y = 0.065
    static_transformStamped.transform.translation.z = 1.111

    #quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    static_transformStamped.transform.rotation.x = 0.000
    static_transformStamped.transform.rotation.y = 0.707
    static_transformStamped.transform.rotation.z = 0.000
    static_transformStamped.transform.rotation.w = 0.707

    #publishes the overhead_camera to the world transformation
    broadcaster.sendTransform(static_transformStamped)
    print(static_transformStamped)
    rospy.spin()

if __name__ == '__main__':

    #just do a static frame 
    rospy.init_node('pnp_overhead_camera_broadcaster')
    pnp = camera_pose()
    pnp.main()
    #main()



