#!/usr/bin/env python3
#try to follow the convention Simon took but with less functions (too lazy to scroll loool)

import time
import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from darknet_ros_msgs.msg import BoundingBoxes, LettuceHypothesis, LettuceHypothesisSet
from image_geometry import PinholeCameraModel


#custom library based of Simons code
from static_model import StaticModel
from lettuce_hypothesis import LettuceHypothesisComplete




#For subscribing to the topic 
OVERHEAD_CAMERA_TOPIC_NAME = '/camera1/usb_cam/image_raw'
OVERHEAD_CAMERA_INFO_TOPIC_NAME = '/camera1/usb_cam/camera_info'
YOLO_BOUNDING_BOXES = '/darknet_ros/bounding_boxes'
BBOX_TO_WORLD_COORDINATES = '/bbox_pixel_to_world_coordinates'



class potato_localiser:
    def __init__(self):
        self.bboxes = None 
        self.camera_model = None
        self.bbox_subscriber = rospy.Subscriber(YOLO_BOUNDING_BOXES, BoundingBoxes, self.receive_bbox)
        self.camera_info_subscriber = rospy.Subscriber(OVERHEAD_CAMERA_INFO_TOPIC_NAME, CameraInfo, self.camera_info_callback)
        self.coordinates_publisher = rospy.Publisher(BBOX_TO_WORLD_COORDINATES, Pose, queue_size = 10)


    def receive_bbox(self, data):
        self.bboxes = data.bounding_boxes


    def camera_info_callback(self, data):
        while self.camera_model == None:
            self.camera_model = PinholeCameraModel()
            self.camera_model.fromCameraInfo(data)
            self.camera_info_subscriber.unregister() #Only subscribe once  
        
        #setup the static camera model
        self.static_model = StaticModel('overhead', self.camera_model)

        

    def main(self):

        image_width = 640
        image_height = 480

        #this will just repeat forever
        while not rospy.is_shutdown():
            hypotheses = []
            #print(self.bboxes)

            if self.bboxes != None:
                for bbox in self.bboxes:
                    label = bbox.Class
                    probability = bbox.probability
                    left_x = bbox.xmin
                    top_y = bbox.ymin
                    width = bbox.xmax - bbox.xmin
                    height = bbox.ymax - bbox.ymin
                    bounding_box = [left_x, top_y, width, height]
                    #print("Bounding box", bounding_box)
                    #Converts to a class called lettuce hypothesiss
                    hypothesis = LettuceHypothesisComplete(bounding_box, image_height, image_width, label, probability)
                    hypothesis = self.static_model.calculate_3D_position(hypothesis)
                    pose = Pose()
                    
                    #only using the x and y coordinates from now
                    pose.position.x = hypothesis.centre3D.point.x
                    pose.position.y = hypothesis.centre3D.point.y
                    print(pose)
                    self.coordinates_publisher.publish(pose)


            
            #except:
            #
            #    print("Nothing yet")
            rospy.sleep(1)
            #rospy.spin()








if __name__ == '__main__':
    rospy.init_node('pixel_to_robot_coordinates', anonymous=True)
    potato = potato_localiser()
    potato.main()