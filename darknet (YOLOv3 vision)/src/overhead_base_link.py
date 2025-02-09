#!/usr/bin/env python3

import tf
import tf2_ros
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion, Transform 


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
    static_transformStamped.transform.translation.x = 0.444
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

    #Publishes a static frame 
    rospy.init_node('overhead_camera_broadcaster')
    main()


