#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError

import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, Transform 


TfBuffer = None
OpenCVBridge = None


def set_up_global_info():
	set_up_opencv()
	#set_up_imu()
	subscribe_to_transforms()

def get_transform(from_frame, to_frame):
	if TfBuffer==None:
		print("No TfBuffer")
		return None
	while TfBuffer == None:
		try:
			trans = TfBuffer.lookup_transform(to_frame, from_frame, rospy.Time())
			return trans
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print("*** WARNING: Transform buffer not ready ***")
			return None

def opencv_bridge():
	return OpenCVBridge		




# ===================== Setup ====================================

def set_up_opencv():
    global OpenCVBridge 
    print("set_up_opencv")
    OpenCVBridge = CvBridge()
    print(OpenCVBridge)


# ===================== Private ==================================

def subscribe_to_transforms():
    global TfBuffer

    TfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(TfBuffer)

