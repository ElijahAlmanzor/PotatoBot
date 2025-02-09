#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import numpy as np
import math
import rtde_receive
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Point
from scipy.spatial.transform import Rotation as R

TCP_WORLD_COORDINATES = '/tcp_world_coordinates'

# Convert Euler angles to Quaternion
def euler_to_quaternion(r):
    yaw, pitch, roll = r
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]

# Convert Quaternion to Euler angles
def quaternion_to_euler(q):
    x, y, z, w = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def main():
    rospy.init_node('ee_base_link', anonymous=True)
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.2.10")
    tcp_publisher = rospy.Publisher(TCP_WORLD_COORDINATES, Point, queue_size=10)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    rpitch = R.from_euler('y', -90, degrees=True)
    rroll = R.from_euler('z', -90, degrees=True)
    RTrv = rpitch * rroll
    rvTR = RTrv.inv()

    while not rospy.is_shutdown():
        pose = rtde_r.getActualTCPPose()
        ROS_pose_x, ROS_pose_y, ROS_pose_z = -pose[1], pose[0], pose[2]
        robot_ori = R.from_rotvec(pose[3:]).as_euler("xyz")
        ori_rviz_rh = rvTR * R.from_euler("xyz", robot_ori)
        ori_rviz = [ori_rviz_rh.as_euler("xyz")[1], ori_rviz_rh.as_euler("xyz")[0], ori_rviz_rh.as_euler("xyz")[2]]
        quat = tf.transformations.quaternion_from_euler(*ori_rviz)

        H, h, w = 0.397, -0.034, 0.038
        angle = np.pi / 2 - ori_rviz[2]
        dx = np.cos(angle) * (H + h) + np.sin(angle) * w
        dy, dz = 0.035, np.sin(angle) * (H + h) - np.cos(angle) * w

        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "base_link"
        static_transformStamped.child_frame_id = "overhead_camera"
        static_transformStamped.transform.translation.x = ROS_pose_x + dx
        static_transformStamped.transform.translation.y = ROS_pose_y + dy
        static_transformStamped.transform.translation.z = ROS_pose_z + dz
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)
        point = Point(x=ROS_pose_x, y=ROS_pose_y, z=ROS_pose_z)
        tcp_publisher.publish(point)
        print("Broadcasting transformation\n", static_transformStamped)
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()
