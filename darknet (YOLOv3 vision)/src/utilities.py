#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped

def xyz_to_stamped_point(xyz, frame_id='camera_link'):
	point = PointStamped()
	point.header.stamp = rospy.Time.now()
	point.header.frame_id = frame_id
	point.point.x = xyz[0]
	point.point.y = xyz[1]
	point.point.z = xyz[2]
	return point    
