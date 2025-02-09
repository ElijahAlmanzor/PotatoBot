#!/usr/bin/env python3
#try to follow the convention Simon took but with less functions (too lazy to scroll loool)

import time
import cv2
import numpy as np
import rospy
import pyrealsense2
import tf2_ros
import tf2_geometry_msgs

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PointStamped, PoseArray
from darknet_ros_msgs.msg import BoundingBoxes, LettuceHypothesis, LettuceHypothesisSet
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge

#custom library based of Simons code
from static_model import StaticModel
from lettuce_hypothesis import LettuceHypothesisComplete




#For subscribing to the topic 
OVERHEAD_CAMERA_TOPIC_NAME = '/camera1/color/image_raw'
OVERHEAD_CAMERA_DEPTH_TOPIC_NAME = '/camera1/depth/image_rect_raw'
OVERHEAD_CAMERA_INFO_TOPIC_NAME = '/camera1/color/camera_info'

YOLO_BOUNDING_BOXES = '/darknet_ros/bounding_boxes'
BBOX_TO_WORLD_COORDINATES = '/bbox_pixel_to_world_coordinates'



class potato_localiser:
    def __init__(self):
        self.bboxes = None 
        self.camera_model = None
        self.bbox_subscriber = rospy.Subscriber(YOLO_BOUNDING_BOXES, BoundingBoxes, self.receive_bbox)
        self.camera_info_subscriber = rospy.Subscriber(OVERHEAD_CAMERA_INFO_TOPIC_NAME, CameraInfo, self.camera_info_callback)
        self.camera_depth_subscriber = rospy.Subscriber(OVERHEAD_CAMERA_DEPTH_TOPIC_NAME, Image, self.camera_depth_callback)
        self.coordinates_publisher = rospy.Publisher(BBOX_TO_WORLD_COORDINATES, PoseArray, queue_size = 10)
        self.bridge = CvBridge()
        self.intrinsics = None
        self.GetIntrinsics()

        self.transform = None
        self.get_world_to_camera_transform()
     

    def get_world_to_camera_transform(self):
        #trans = get_transform('world','overhead_camera')

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)  

        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform('base_link', 'overhead_camera', rospy.Time())
                #print(trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(1)
                continue

            if trans is not None:
                break


        self.transform = trans  
       

    def transformed_point(self,x, y, z):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = 'overhead_camera'
        point.point.x = x *0.001
        point.point.y = y *0.001
        point.point.z = z *0.001

        point_in_world = tf2_geometry_msgs.do_transform_point(point, self.transform)
        return point_in_world


    def convert_camera_to_world_frame(self,point):
        return self.transformed_point(point[0],point[1],point[2])


    def receive_bbox(self, data):
        self.bboxes = data.bounding_boxes


    def camera_depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
    def GetIntrinsics(self):
        print("waiting for the camera")
        for i in range(3):
            rospy.sleep(1)
            print("countdown", 3-(i+1))
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

    def camera_info_callback(self, data):
     
        self.camera_model = data
    

    def convert_depth_to_phys_coord_using_realsense(self, x, y, depth):  

        #return coordinates of the object coordinates to the cameras frame coordinates  
        result = pyrealsense2.rs2_deproject_pixel_to_point(self.intrinsics, [x, y], depth)  
     
        return result[2], -result[0], -result[1]


    def main(self):
        
        

        image_width = self.camera_model.width
        image_height = self.camera_model.height


     
        while not rospy.is_shutdown():
            self.get_world_to_camera_transform()

            potato_locations = PoseArray()
            potato_locations.header.frame_id = "potato"

            bboxes_in_order = []
            try:
                #Extract bounding boxes
                self.bboxes.sort(key=lambda box: box.xmin, reverse=False)
           
                for bbox in self.bboxes:
                   
                    label = bbox.Class
                    probability = bbox.probability
                    left_x = bbox.xmin
                    top_y = bbox.ymin
                    width = bbox.xmax - bbox.xmin
                    height = bbox.ymax - bbox.ymin
                    bounding_box = [left_x, top_y, width, height]
                    x_centre = int(left_x + width/2)
                    y_centre = int(top_y + height/2)
   
                    depth = self.depth_image[y_centre,x_centre]
                    print("Object in camera pixels", x_centre,y_centre,depth)


                    xyz_camera_frame = self.convert_depth_to_phys_coord_using_realsense(x_centre,y_centre, depth)
                    print("object in camera frame", xyz_camera_frame, "in mm")

                    
                    xyz_world_frame = self.convert_camera_to_world_frame(xyz_camera_frame)
                    print("object in the world frame", xyz_world_frame, "in m")
                    



                    pose = Pose()
                    pose.position.x = xyz_world_frame.point.x 
                    pose.position.y = xyz_world_frame.point.y 
                    pose.position.z = xyz_world_frame.point.z 
                    potato_locations.poses.append(pose)
                
                
                
                
            except:
                print("no bboxes")

   
            self.coordinates_publisher.publish(potato_locations)
            rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('pixel_to_robot_coordinates', anonymous=True)
    potato = potato_localiser()
    potato.main()