#!/usr/bin/env python3

import time
import os
import numpy as np
import rospy
import cv2
from copy import deepcopy
from math import pi
from kg_robot import kg_robot as kgr
from sensor_msgs.msg import Image, Pose, Point
from cv_bridge import CvBridge

topics = {
    "bbox": "/bbox_pixel_to_world_coordinates",
    "tcp": "/tcp_world_coordinates",
    "camera": "/camera1/color/image_raw"
}

class PlantOperation:
    def __init__(self):
        print("\n------------ Configuring Spudnik, the Potato Planter -------------\n")
        self.spudnik = kgr.kg_robot(port=30010, db_host="192.168.2.10")
        self.bridge = CvBridge()
        self.digital_pin, self.valve_off, self.valve_on = 0, 1, 0
        self.acc_val, self.vel_val = 0.5, 0.5
        self.bbox_location, self.color_image, self.tcp_location = None, None, None
        self.row_id, self.column_id = 0, 0
        self.ind_times = []
        self.trajectory_points, self.trajectory_start_time = [], None
        self.record_trajectory = False
        self.setup_starting_poses()
        
        rospy.Subscriber(topics["bbox"], Pose, self.pose_callback)
        rospy.Subscriber(topics["camera"], Image, self.camera_color_callback)
        rospy.Subscriber(topics["tcp"], Point, self.tcp_callback)

    def pose_callback(self, data):
        self.bbox_location = data

    def camera_color_callback(self, data):
        self.color_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def tcp_callback(self, data):
        self.tcp_location = data
        if self.record_trajectory:
            self.trajectory_points.append([data.x, data.y, data.z, time.time() - self.trajectory_start_time])

    def reset_trajectory_tracking(self):
        self.trajectory_start_time, self.trajectory_points, self.record_trajectory = None, [], False

    def start_trajectory_tracking(self):
        self.trajectory_start_time, self.record_trajectory = time.time(), True

    def setup_starting_poses(self):
        self.home_pose = Pose(position=Point(-0.15, -0.90, 0.453))
        self.start_plant_pose = Pose(position=Point(0.4, 0.3, 0.2))

    def drop_down_pick_and_place(self, bbox):
        pose_above = deepcopy(bbox)
        pose_above.position.z += 0.2
        pose_above_robot = self.spudnik.ros_to_robot_joint_pose_transform_rotate(pose_above, 0)
        self.spudnik.movej(pose_above_robot, acc=self.acc_val, vel=self.vel_val)

        pick_pose = deepcopy(pose_above)
        pick_pose.position.z -= 0.17
        pick_pose_robot = self.spudnik.ros_to_robot_joint_pose_transform_rotate(pick_pose, 0)
        self.spudnik.movej(pick_pose_robot, acc=self.acc_val, vel=self.vel_val)
        self.spudnik.set_digital_out(self.digital_pin, self.valve_on)
        rospy.sleep(0.1)

        self.spudnik.movej(pose_above_robot, acc=self.acc_val, vel=self.vel_val)
        home_plant_robot = self.spudnik.ros_to_robot_joint_pose_transform_rotate(self.start_plant_pose, 0)
        self.spudnik.movej(home_plant_robot, acc=self.acc_val, vel=self.vel_val)

        place_above = deepcopy(self.start_plant_pose)
        place_above.position.x += self.row_id * 0.15
        place_above.position.y += self.column_id * -0.2
        place_above_robot = self.spudnik.ros_to_robot_joint_pose_transform_rotate(place_above, 0)
        self.spudnik.movej(place_above_robot, acc=self.acc_val, vel=self.vel_val)

        place = deepcopy(place_above)
        place.position.z -= 0.64
        place_robot = self.spudnik.ros_to_robot_joint_pose_transform_rotate(place, pick_pose.orientation.x)
        self.spudnik.movej(place_robot, acc=self.acc_val, vel=self.vel_val)

        self.spudnik.set_digital_out(self.digital_pin, self.valve_off)
        rospy.sleep(0.2)
        self.spudnik.movej(home_plant_robot, acc=self.acc_val, vel=self.vel_val)

    def main(self):
        self.spudnik.set_digital_out(self.digital_pin, self.valve_off)
        home_pose_robot = self.spudnik.ros_to_robot_pose_transform_no_rotation(self.home_pose)
        self.spudnik.movel(home_pose_robot, acc=self.acc_val, vel=self.vel_val)

        print("Waiting for the potato location...")
        while not self.bbox_location:
            rospy.sleep(0.1)
        
        start_time, potato_counter = time.time(), 0

        while not rospy.is_shutdown():
            if not self.bbox_location:
                print("No more potatoes detected, finishing.")
                break

            self.spudnik.movel(home_pose_robot, acc=self.acc_val, vel=self.vel_val)
            rospy.sleep(0.5)
            hypothesis = deepcopy(self.bbox_location)
            self.drop_down_pick_and_place(hypothesis)
            
            elapsed = time.time() - start_time
            print(f"Planted {potato_counter + 1} potatoes in {elapsed:.2f} seconds")
            self.ind_times.append(elapsed / (potato_counter + 1))
            potato_counter += 1

            save_path_time = os.path.join(os.path.expanduser('~'), 'pp_ws', 'src', 'planter', 'times', 'plant_times.npy')
            np.save(save_path_time, self.ind_times)

        self.spudnik.movel(home_pose_robot, acc=self.acc_val, vel=self.vel_val)
        print("Goodbye")
        self.spudnik.close()

if __name__ == '__main__':
    rospy.init_node('plant_operation', anonymous=True)
    plant = PlantOperation()
    plant.main()
