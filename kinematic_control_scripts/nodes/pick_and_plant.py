#!/usr/bin/env python3

import os
import time
import socket
import serial
import numpy as np
import rospy
import cv2
import pyrealsense2
import tf2_ros
import tf2_geometry_msgs

from math import pi
from copy import deepcopy
from kg_robot import kg_robot as kgr
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import TransformStamped, Pose, PoseArray, Point, PointStamped
from sensor_msgs.msg import Image, CameraInfo

# ROS Topics
BBOX_TO_WORLD_COORDINATES = '/bbox_pixel_to_world_coordinates'
TCP_WORLD_COORDINATES = '/tcp_world_coordinates'
OVERHEAD_CAMERA_COLOR_TOPIC_NAME = '/camera1/color/image_raw'


class plant_operation:
    def __init__(self):
        print("------------ Configuring Spudnik, the Potato Planter -------------\r\n")
        
        # Robot connection
        self.spudnik = kgr.kg_robot(port=30010, db_host="192.168.2.10")

        # Movement parameters
        self.acc_val = 3
        self.vel_val = 3
        self.digital_pin = 0
        self.valve_off = 0
        self.valve_on = 1

        # Image processing
        self.bridge = CvBridge()  # ROS image message conversion

        # State variables
        self.home_pose = None
        self.start_plant_pose = None
        self.color_image = None
        self.bbox_location = None
        self.angle = None
        self.angled_picking = False
        self.angled_picking2 = False

        # TCP tracking & trajectory data
        self.tcp_location = None
        self.record_trajectory = False
        self.trajectory_points = []  # List of [x, y, z, time]
        self.trajectory_start_time = None

        # Crop row & column tracking
        self.row_id = 0
        self.column_id = 0
        self.ind_times = []
        self.angled_ind_times = []

        # ROS Subscribers
        rospy.Subscriber(BBOX_TO_WORLD_COORDINATES, Pose, self.pose_callback)
        rospy.Subscriber(OVERHEAD_CAMERA_COLOR_TOPIC_NAME, Image, self.camera_color_callback)
        rospy.Subscriber(TCP_WORLD_COORDINATES, Point, self.tcp_callback)

        # Initialize robot poses
        self.setup_starting_poses()

    def pose_callback(self, data):
        self.bbox_location = data
    
    def camera_color_callback(self, data):
        self.color_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def tcp_callback(self, data):
        self.tcp_location = data

        
        if self.record_trajectory:
            current_time = time.time() - self.trajectory_start_time
            point = self.tcp_location

            location = []
            location.append(point.x)
            location.append(point.y)
            location.append(point.z)
            #point with time 
            location.append(current_time)
            self.trajectory_points.append(location)

    def reset_trajectory_tracking(self):
        self.trajectory_start_time = 0
        self.trajectory_points = []
        self.record_trajectory = False

    def start_trajectory_tracking(self):
        self.trajectory_start_time = time.time()
        self.record_trajectory = True




    def setup_starting_poses(self):
        #First move to a default position
        self.home_pose = Pose()
        self.home_pose.position.x = -0.15
        self.home_pose.position.y = -0.90
        self.home_pose.position.z =  0.453


        #Move to the other home position to check for other potatoes
        self.home_pose_2 = Pose()
        self.home_pose_2.position.x = -0.0
        self.home_pose_2.position.y = -0.90
        self.home_pose_2.position.z =  0.453

        #First move to an end position
        self.photo_pose = Pose()
        self.photo_pose.position.x = 0.83
        self.photo_pose.position.y = -0.25
        self.photo_pose.position.z =  0.85

        #Pre-plant pose (for inbuilt inverse kinematics functionality)
        self.home_plant_pose = Pose()
        self.home_plant_pose.position.x = 0.5
        self.home_plant_pose.position.y = -0.2
        self.home_plant_pose.position.z =  0.403 #Just so I ensure it doesnt crash into the box


        #Pose for planting
        self.start_plant_pose = Pose()
        self.start_plant_pose.position.x = 0.4
        self.start_plant_pose.position.y = 0.3
        self.start_plant_pose.position.z =  0.2 


    def drop_down_pick_and_place(self, bbox):
        """Moves the robot to pick a potato using a drop-down approach."""

        # Move to a position above the potato
        pose_above = deepcopy(bbox)
        pose_above.position.z += 0.25  # Offset to avoid collision
        pose_above.position.x -= 0.01  # Small lateral adjustment

        pose_above_robot = self.spudnik.ros_to_robot_joint_pose_transform_rotate(pose_above, 0)
        self.spudnik.movej(pose_above_robot, acc=self.acc_val, vel=self.vel_val)

        # Move down to pick the potato
        pick_pose = deepcopy(pose_above)
        pick_pose.position.z -= 0.212

        pick_pose_robot = self.spudnik.ros_to_robot_joint_pose_transform_rotate(pick_pose, 0)
        self.spudnik.movej(pick_pose_robot, acc=self.acc_val, vel=self.vel_val)

        # Activate gripper
        self.spudnik.set_digital_out(self.digital_pin, self.valve_on)
        rospy.sleep(0.1)

        # Move back up
        self.spudnik.movej(pose_above_robot, acc=self.acc_val, vel=self.vel_val)

        

    def angled_pick_and_plant(self, bbox):
        """Moves the robot to pick a potato at an angle."""

        # Move to a position above the potato
        pose_above = deepcopy(bbox)
        pose_above.position.z += 0.1
        pose_above.position.x -= 0.15

        pose_above_robot = self.spudnik.ros_to_robot_pose_transform_at_angle(pose_above)
        self.spudnik.movel(pose_above_robot, acc=self.acc_val, vel=self.vel_val)

        rospy.sleep(0.1)
        self.angle = self.bbox_location.orientation.x
        print(f"Angle detected: {self.angle}")

        # Move down to pick the potato
        pick_pose = deepcopy(pose_above)
        pick_pose.position.z -= 0.06
        pick_pose.position.x += 0.115

        pre_pick_pose_robot = self.spudnik.ros_to_robot_pose_transform_at_angle(pick_pose)
        self.spudnik.movel(pre_pick_pose_robot, acc=self.acc_val, vel=self.vel_val)

        # Activate gripper
        self.spudnik.set_digital_out(self.digital_pin, self.valve_on)
        rospy.sleep(0.3)

        # Move back up
        pose_above.position.z += 0.2
        pose_above.position.x -= 0.15

        pose_above_robot = self.spudnik.ros_to_robot_pose_transform_no_rotation(pose_above)
        self.spudnik.movel(pose_above_robot, acc=self.acc_val, vel=self.vel_val)

    def angled_pick_and_plant_2(self, bbox):
        """Moves the robot to pick a potato at an angle using an alternative approach."""

        # Move to a position above the potato
        pose_above = deepcopy(bbox)
        pose_above.position.z += 0.1
        pose_above.position.y -= 0.01
        pose_above.position.x -= 0.15

        pose_above_robot = self.spudnik.ros_to_robot_pose_transform_at_angle(pose_above)
        self.spudnik.movel(pose_above_robot, acc=self.acc_val, vel=self.vel_val)
        
        rospy.sleep(0.1)
        self.angle = self.bbox_location.orientation.x
        print(f"Detected angle: {self.angle}")

        # Move down to pick the potato
        pick_pose = deepcopy(pose_above)
        pick_pose.position.z -= 0.073
        pick_pose.position.x += 0.145

        pre_pick_pose_robot = self.spudnik.ros_to_robot_pose_transform_at_angle(pick_pose)
        self.spudnik.movel(pre_pick_pose_robot, acc=self.acc_val, vel=self.vel_val)

        # Activate gripper
        self.spudnik.set_digital_out(self.digital_pin, self.valve_on)
        rospy.sleep(0.1)

        # Move back up
        pose_above.position.z += 0.2
        pose_above.position.x -= 0.15
        pose_above_robot = self.spudnik.ros_to_robot_pose_transform_no_rotation(pose_above)
        self.spudnik.movel(pose_above_robot, acc=self.acc_val, vel=self.vel_val)``

        

    def home_plant(self, pick_pose):
        """Moves the robot to a home planting position and plants the potato."""

        # Move to the pre-plant position (ensuring safe movement)
        home_plant = deepcopy(self.home_plant_pose)
        home_plant_robot = self.spudnik.ros_to_robot_joint_pose_transform_rotate(home_plant, 0)
        self.spudnik.movej(home_plant_robot, acc=self.acc_val, vel=self.vel_val)

        # Move to the planting position
        place = deepcopy(self.start_plant_pose)
        place.position.z -= 0.66  # Drop the potato
        place.position.x += self.row_id * 0.2
        place.position.y += self.column_id * -0.1

        place_robot = self.spudnik.ros_to_robot_joint_pose_transform_rotate(place, pick_pose.orientation.x)
        self.spudnik.movej(place_robot, acc=self.acc_val, vel=self.vel_val)

        # Deactivate gripper and move back
        self.spudnik.set_digital_out(self.digital_pin, self.valve_off)
        rospy.sleep(0.2)
        self.spudnik.movej(home_plant_robot, acc=self.acc_val, vel=self.vel_val)


    def main(self):
        """Main function for robotic potato picking and planting."""

        # Experiment Variables
        self.row_id, self.column_id = 0, 0
        experiment_number = 20
        is_continue = False
        continue_number = 0

        # File naming
        name_suffix = f"{experiment_number}{continue_number}_" if is_continue else f"{experiment_number}"
        name_time = f"{name_suffix}times.npy"
        angled_name_time = f"{name_suffix}angledtimes.npy"
        start_image_save = f"{name_suffix}_start.png"

        # Disable gripper and move to home position
        self.spudnik.set_digital_out(self.digital_pin, self.valve_off)
        home_pose_robot = self.spudnik.ros_to_robot_pose_transform_no_rotation(self.home_pose)
        home_pose_robot_2 = self.spudnik.ros_to_robot_pose_transform_no_rotation(self.home_pose_2)
        self.spudnik.movel(home_pose_robot, acc=self.acc_val, vel=self.vel_val)

        # Save the initial image
        rospy.sleep(1)
        save_start_image_path = os.path.join(
            os.path.expanduser('~'), 'pp_ws', 'src', 'planter', 
            'planter_kg_control', 'start_end_image', start_image_save
        )
        cv2.imwrite(save_start_image_path, cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR))

        # Start time tracking
        start_time = time.time()
        potato_counter = 0

        while not rospy.is_shutdown():
            individual_time = time.time()

            # Start trajectory tracking
            self.start_trajectory_tracking()

            # Move to home position
            self.spudnik.movel(home_pose_robot, acc=self.acc_val, vel=self.vel_val)
            self.bbox_location = None  # Reset bbox location

            # Wait for new potato locations
            rospy.sleep(2)

            if not self.bbox_location:
                print("No more potatoes detected.")
                self.spudnik.movel(home_pose_robot, acc=self.acc_val, vel=self.vel_val)
                break

            # Move to an alternative home position if no potatoes found
            if not self.bbox_location:
                self.spudnik.movel(home_pose_robot_2, acc=self.acc_val, vel=self.vel_val)
                self.angled_picking = True
                rospy.sleep(2)

            # If a potato is close to the edge, use angled picking
            elif self.bbox_location.orientation.z < 10 or self.bbox_location.orientation.w < 40:
                self.angled_picking2 = True

            # If no potatoes after angled picking, exit loop
            if not self.bbox_location and self.angled_picking:
                break

            # Store current bounding box location
            hypothesis = deepcopy(self.bbox_location)

            if not hypothesis:
                print("No more potatoes left.")
                break

            # Execute appropriate pick-and-place strategy
            if self.angled_picking:
                self.angled_pick_and_plant(hypothesis)
                hypothesis.orientation.x = self.angle
                self.angled_picking = False
            elif self.angled_picking2:
                self.angled_pick_and_plant_2(hypothesis)
                hypothesis.orientation.x = self.angle
                self.angled_picking2 = False
            else:
                self.drop_down_pick_and_place(hypothesis)

            # Plant the potato
            self.home_plant(hypothesis)
            self.angle = 0

            # Time tracking
            elapsed_picking_time = time.time() - individual_time
            print(f"Time taken for this potato: {elapsed_picking_time:.2f} seconds")

            if self.angled_picking or self.angled_picking2:
                self.angled_ind_times.append(elapsed_picking_time)
            else:
                self.ind_times.append(elapsed_picking_time)

            # Save trajectory data
            trajectory_save_path = os.path.join(
                os.path.expanduser('~'), 'pp_ws', 'src', 'planter', 
                'planter_kg_control', 'trajectories', f"{self.row_id}.npy"
            )
            with open(trajectory_save_path, "wb") as f:
                np.save(f, self.trajectory_points)

            # Reset trajectory tracking
            self.reset_trajectory_tracking()

            # Save experiment times
            time_save_path = os.path.join(
                os.path.expanduser('~'), 'pp_ws', 'src', 'planter', 
                'planter_kg_control', 'times', name_time
            )
            with open(time_save_path, "wb") as f:
                np.save(f, self.ind_times)

            # Save angled experiment times
            angled_time_save_path = os.path.join(
                os.path.expanduser('~'), 'pp_ws', 'src', 'planter', 
                'planter_kg_control', 'times', angled_name_time
            )
            with open(angled_time_save_path, "wb") as f:
                np.save(f, self.angled_ind_times)

            # Adjust row and column positions for planting
            self.row_id += 1
            if self.row_id % 5 == 0:
                self.row_id = 0
                self.column_id += 1

            potato_counter += 1

        # If no potatoes were picked, exit
        if not potato_counter:
            self.spudnik.close()
            return

        # Final time tracking
        elapsed_time = time.time() - start_time
        time_per_potato = elapsed_time / potato_counter

        print(f"Total potatoes picked: {potato_counter}")
        print(f"Total planting time: {elapsed_time:.2f} seconds")
        print(f"Time per potato: {time_per_potato:.2f} seconds")

        # Move back home and close the robot
        self.spudnik.movel(home_pose_robot, acc=self.acc_val, vel=self.vel_val)
        print("Goodbye")
        self.spudnik.close()


        
if __name__ == '__main__':
    rospy.init_node('plant_operation', anonymous=True)
    plant = plant_operation()
    plant.main()
