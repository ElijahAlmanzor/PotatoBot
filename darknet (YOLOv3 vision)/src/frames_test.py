#!/usr/bin/env python

import tf
import tf2_ros
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion, Transform 


def main():

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)  

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('robot_base', 'overhead_camera', rospy.Time())
            print(trans)
            rospy.sleep(10)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(1)
            continue

if __name__ == '__main__':

    #just do a static frame 
    rospy.init_node('test_conversion')
    main()


