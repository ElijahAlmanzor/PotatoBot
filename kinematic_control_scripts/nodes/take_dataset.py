#!/usr/bin/env python3
import time
import serial
from math import pi
import numpy as np
import numpy
import socket
import rospy
from kg_robot import kg_robot as kgr
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from copy import deepcopy
from geometry_msgs.msg import TransformStamped, Pose, PoseArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#Topics to subsribe to
BBOX_TO_WORLD_COORDINATES = '/bbox_pixel_to_world_coordinates'
###
#camera 2 for the camera in the gripper
CAMERA_TOPIC_NAME = '/camera1/color/image_raw'

#some global coordinates... for now ;)
bbox_location = None
camera_image = None
OpenCVBridge = None


def pose_callback(data):
    global bbox_location
    bbox_location = data

def image_callback(data):
    global camera_image
    camera_image = data

def set_up_opencv():
	global OpenCVBridge
	OpenCVBridge = CvBridge()

def main():
    global bbox_location
    global camera_image
    global OpenCVBridge
    set_up_opencv() 


    rospy.init_node('controller', anonymous=True)
    print("------------Configuring Planter Planter (PP)-------------\r\n")
    burt = kgr.kg_robot(port=30010,db_host="192.168.2.10")
    #burt = kgr.kg_robot(port=30010,ee_port="COM32",db_host="192.168.1.51")

    #subscribe to the bbox 3D pixel topic
    rospy.Subscriber(BBOX_TO_WORLD_COORDINATES, PoseArray, pose_callback)
    rospy.Subscriber(CAMERA_TOPIC_NAME, Image, image_callback)
    #robot_pose = burt.getl()
    #ROS_pose = burt.robot_to_ros_pose_transform(robot_pose)
    #print("Current Pose in ROS", ROS_pose)
    #burt.close()


    #First move to a default position
    home_pose = Pose()
    home_pose.position.x = -0.010
    home_pose.position.y = -0.75
    home_pose.position.z =  0.75
    home_pose.orientation.x = 0.0131978842248
    home_pose.orientation.y = -0.0317778494676
    home_pose.orientation.z = -0.999377232394
    home_pose.orientation.w = 0.00781866405548
    home_pose_robot = burt.ros_to_robot_pose_transform_no_rotation(home_pose)
    burt.movel(home_pose_robot)
    #home_pose_robot = burt.ros_to_robot_joint_pose_transform_rotate(home_pose, 90)
    #burt.movej(place_above_robot)
    print("moving to the home position")
    #rospy.sleep(1)
    
    
    coordinates = []

    #Start coordinates
    #x_start = 650
    x_start = -0.14
    y_start = -0.95
    z_start = 0.5

    #End coordinates
    #x_end = -650
    x_end = 0.120
    y_end = -0.550
    z_end = 0.8

    number_of_points = 10
    x_step = (x_end - x_start)/5
    y_step = (y_end - y_start)/10
    z_step = (z_end - z_start)/4
    #print("the steps", x_step, y_step, z_step)
    #rospy.sleep(10)
    mm_to_m = float(1000)
    z_c = np.linspace(z_start,z_end, num = 3)
    y_c = np.linspace(y_start,y_end, num = 3)
    x_c = np.linspace(x_start,x_end, num = 3)
    #Determines the grid for where to move within the workspace
    for z in z_c:
        for x in x_c:
            for y in y_c:
                coordinate = (x,y,z)
                coordinates.append(coordinate)

    n = 502
    filename = "image_"
    for coord in coordinates:
        pose = Pose()
        pose.position.x = coord[0]
        pose.position.y = coord[1]
        pose.position.z =  coord[2]
        pose.orientation.x = 0.0131978842248
        pose.orientation.y = -0.0317778494676
        pose.orientation.z = -0.999377232394
        pose.orientation.w = 0.00781866405548
        
        if (camera_image==None):
            rospy.loginfo("ERROR: No camera image received yet on " + CAMERA_TOPIC_NAME)
        else:
            try:
                print("Save Image number", n)
                cv2_img = OpenCVBridge.imgmsg_to_cv2(camera_image, "bgr8")
                full_filename = filename + str(n) + ".jpg"
                cv2.imwrite(full_filename, cv2_img)
            except CvBridgeError:#, e:
                print("some error")
        n += 1
        
        pose_robot = burt.ros_to_robot_pose_transform_no_rotation(pose)
        burt.movel(pose_robot, acc = 2, vel= 2)
        print(pose)
        rospy.sleep(0.2)



    
    
    print("Goodbye")
    burt.close()

        
if __name__ == '__main__':
    main()
