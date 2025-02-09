#!/usr/bin/env python3

import numpy as np
import time
import serial
import socket
import math
from math import pi

from kg_robot_dashboard import kg_robot_dashboard as kgrd
from waypoints import waypoints as wp
from teach_mode import teach_mode as tm

#ROS stuff
from geometry_msgs.msg import TransformStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy.spatial.transform import Rotation as R


class kg_robot():
    def __init__(self, port=False, ee_port=False, db_host=False):
        self.port = port

        self.ee_port = ee_port
        self.db_host = db_host
        if db_host!=False:
            self.dashboard = kgrd.kg_robot_dashboard(host=self.db_host)
            self.dashboard.init()

        #init attached modules
        self.teach_mode = tm.teach_mode(self)



        #init ur5 connection
        self.open=False
        if port!=False:
            self.host = "192.168.2.11"

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port)) # Bind to the port
            s.listen(5) # Now wait for client connection.
            self.c, self.addr = s.accept() # Establish connection with client.
            print("Connected to UR10\r\n")
            self.open=True

            #self.home(pose = wp.burt_homej, wait=False)


        #init gripper connection and update robot tcp
        if ee_port!=False:
            self.ee = serial.Serial(self.ee_port, 9600)  # open serial port
            while self.ee.isOpen()==False:
                print("Waiting for hand")
            #print("Serial port opened :)")

            self.ee.send_break()
            time.sleep(1) # This is needed to allow MBED to send back command in time!
            ipt = bytes.decode(self.ee.readline())
            print("Connected to",ipt)

            if port!=False:
                if ipt=="Rotary Gripper\r\n":
                    self.set_tcp(wp.rotary_tcp)
                    self.set_payload(1.8)
                elif ipt=="Pincher Gripper\r\n":
                    self.set_tcp(wp.pincher_tcp)
                    self.set_payload(0.5)
                elif ipt == "ElectroMag Gripper\r\n":
                    self.set_tcp(wp.magnet_tcp) 
                    self.set_payload(0.5)
                else:
                    print("NO GRIPPER DETECTED")

        return



    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #
    #                                                                      Communications
    #
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def socket_ping(self):
        msg = "No message from robot"
        prog = self.format_prog(40)
        try:
            # Send formatted CMD
            self.c.send(str.encode(prog))
            # Wait for reply
            while(msg!='robot_ready'):
                msg=bytes.decode(self.c.recv(1024))
            
        except socket.error as socketerror:
            print(".......................Some kind of error :(.......................")
            input("press enter to continue")
        return msg
    
    def socket_send(self, prog):
        msg = "No message from robot"
        try:
            # Send formatted CMD
            self.c.send(str.encode(prog))
            # Wait for reply
            if prog[-3]=='0':
                msg=bytes.decode(self.c.recv(1024))
                if msg=="No message from robot" or msg=='':
                    print(".......................Robot disconnected :O.......................")
                    input("press enter to continue")

        except socket.error as socketerror:
            print(".......................Some kind of error :(.......................")
            input("press enter to continue")
        return msg

    def format_prog(self,CMD,pose=[0,0,0,0,0,0],acc=0.1,vel=0.1,t=0,r=0,w=True):
        wait=0
        if w==False:
            wait=1
        return "({},{},{},{},{},{},{},{},{},{},{},{})\n".format(CMD,pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],acc,vel,t,r,wait)

    def serial_send(self,cmd,var,wait):
        ipt = ""
        self.ee.reset_input_buffer()
        self.ee.write(str.encode(cmd+chr(var+48)+"\n"))
        #wait for cmd acknowledgement
        while True:
            ipt = bytes.decode(self.ee.readline())
            #print("gripper data: ", ipt)
            if ipt == "received\r\n":
                break
        #wait for cmd completion
        if wait==True:
            while True:
                ipt = bytes.decode(self.ee.readline())
                #print("gripper data: ", ipt)
                if ipt == "done\r\n":
                    #print("Completed gripper CMD")
                    break
        return ipt

    def decode_msg(self,prog):
        msg = self.socket_send(prog)
        #print "recieved: ",msg

        # Decode Pose or Joints from UR
        current_position = [0,0,0,0,0,0]
        data_start = 0
        data_end = 0
        n = 0
        x = 0
        while x < len(msg):
            if msg[x]=="," or msg[x]=="]" or msg[x]=="e":
                data_end = x
                current_position[n] = float(msg[data_start:data_end])
                if msg[x]=="e":
                    current_position[n] = current_position[n]*math.pow(10,float(msg[x+1:x+4]))
                    #print "e", msg[x+1:x+4]
                    #print "e", int(msg[x+1:x+4])
                    if n < 5:
                        x = x+5
                        data_start = x
                    else:
                        break
                n=n+1
            if msg[x]=="[" or msg[x]==",":
                data_start = x+1
            x = x+1

        return current_position

    
    def close(self):
        """
        close connection to robot and stop internal thread
        """
        try:
            self.ee.reset_output_buffer()  # Close gripper
        except:
            # No gripper connected
            pass
        if self.open==True:
            prog = self.format_prog(100)
            print(self.socket_send(prog))
            self.c.close()
        
        if self.db_host!=False:
            if self.dashboard.open==True:
                self.dashboard.c.close()


    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #
    #                                                                       UR5 Commands
    #
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # CMD       Description                                         Reply
    # 0         joint move in linear space                          confirmation
    # 1         move in joint space                                 confirmation
    # 2         pose move in linear space                           confirmation
    # 3         pose move relative to current position              confirmation
    # 4         force move in single axis                           confirmation
    #
    # 10        get current pose                                    pose
    # 11        get current jonts                                   joints
    # 12        get inverse kin of sent pose                        joints
    # 13        get transform from current pose to sent pose        pose
    # 14        get force vector                                    pose
    # 15        get force magnitude                                 float
    #
    # 20        set tool centre point (tcp)                         confirmation
    # 21        set payload                                         confirmation
    #
    # 100       close socket on robot                               confirmation

    def ping(self):
        """
        ping robot
        """
        return self.socket_ping()

    def movejl(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        joint move in linear space
        """
        prog = self.format_prog(0,pose=pose,acc=acc,vel=vel,t=min_time,r=radius,w=wait)
        return self.socket_send(prog)

    def movej(self, joints, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        move to joint positions
        """
        prog = self.format_prog(1,pose=joints,acc=acc,vel=vel,t=min_time,r=radius,w=wait)
        return self.socket_send(prog)

    def movej_rel(self, joints, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        move joint positions by 'joints'
        """
        demand_joints = self.getj()
        for i in range(0,6):
            demand_joints[i]+=joints[i]
        prog = self.format_prog(1,pose=demand_joints,acc=acc,vel=vel,t=min_time,r=radius,w=wait)
        return self.socket_send(prog)

    def movel(self, pose, acc=0.1, vel=0.1, min_time=0, radius=0, wait=True):
        """
        pose move in linear space
        """
        prog = self.format_prog(2,pose=pose,acc=acc,vel=vel,t=min_time,r=radius,w=wait)
        return self.socket_send(prog)

    def servoj(self, pose, vel=0.5, control_time=0.1, lookahead_time=0.001, gain=1000, stop=False):
        """
        pose move in linear space
        """
        prog = self.format_prog(5,pose=pose,acc=vel,vel=control_time,t=lookahead_time,r=gain,w=stop)
        return self.socket_send(prog)

    def servoc(self, pose, acc=0.5, vel=0.5, radius=0.001, stop=False):
        """
        pose move in linear space
        """
        prog = self.format_prog(9,pose=pose,acc=acc,vel=vel,r=radius,w=stop)
        return self.socket_send(prog)

    def speedl(self, pose, acc=1, blocking_time=0, wait=True):
        """
        set speed in linear space, blocking time sets how long function runs (robot will stop after) if 0 will return after reaching vel
        """
        prog = self.format_prog(6,pose=pose,acc=acc,t=blocking_time,w=wait)
        return self.socket_send(prog)

    def speedj(self, joints, acc=0.5, blocking_time=0, wait=True):
        """
        set joint speed, blocking time sets how long function runs (robot will stop after) if 0 will return after reaching vel
        """
        prog = self.format_prog(7,pose=joints,acc=acc,t=blocking_time,w=wait)
        return self.socket_send(prog)

    def stopl(self, acc, wait=True):
        """
        decelerate in linear space
        """
        prog = self.format_prog(8,acc=acc,w=wait)
        return self.socket_send(prog)

    def movep(self, pose, acc=0.5, vel=0.5, min_time=0.1, radius=0.001, wait=False):
        """
        pose move in linear space
        """
        prog = self.format_prog(5,pose=pose,acc=acc,vel=vel,t=min_time,r=radius,w=wait)
        return self.socket_send(prog)

    def home(self, pose=None, type='j', acc=0.5, vel=0.5, wait=True):
        """
        move to home position, default joint space
        """
        if type == 'j':
            if pose!=None:
                self.homej = pose
            prog = self.format_prog(1,pose=self.homej,acc=acc,vel=vel,w=wait)
        elif type == 'l':
            if pose!=None:
                self.homel = pose
            prog = self.format_prog(0,pose=self.homel,acc=acc,vel=vel,w=wait)
        return self.socket_send(prog)

    def translatel_rel(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        translate relative to position in linear space
        """
        self.demand_pose = self.getl()
        self.demand_pose[0]+=pose[0]
        self.demand_pose[1]+=pose[1]
        self.demand_pose[2]+=pose[2]
        return self.movel(self.demand_pose,acc=acc,vel=vel,min_time=min_time,radius=radius,wait=wait)

    def translatejl_rel(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        translate relative to position in linear space using joint move
        """
        self.demand_pose = self.getl()
        self.demand_pose[0]+=pose[0]
        self.demand_pose[1]+=pose[1]
        self.demand_pose[2]+=pose[2]
        return self.movejl(self.demand_pose,acc=acc,vel=vel,min_time=min_time,radius=radius,wait=wait)

    def translatel(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        translate to position in linear space
        """
        self.demand_pose = self.getl()
        self.demand_pose[0]=pose[0]
        self.demand_pose[1]=pose[1]
        self.demand_pose[2]=pose[2]
        return self.movel(self.demand_pose,acc=acc,vel=vel,min_time=min_time,radius=radius,wait=wait)

    def translatejl(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        translate to position in linear space using joint move
        """
        self.demand_pose = self.getl()
        self.demand_pose[0]=pose[0]
        self.demand_pose[1]=pose[1]
        self.demand_pose[2]=pose[2]
        return self.movejl(self.demand_pose,acc=acc,vel=vel,min_time=min_time,radius=radius,wait=wait)

    def movel_tool(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        linear move in tool space
        """
        prog = self.format_prog(3,pose=pose,acc=acc,vel=vel,t=min_time,r=radius,w=wait)
        return self.socket_send(prog)

    def force_move(self, axis, acc=0.05, vel=0.05, min_time=0, force=50, wait=True):
        """
        move along axis with a maximum force, e.g. axis = [0,y_dist,0]
        """
        prog = self.format_prog(4,pose=axis+[0,0,0],acc=acc,vel=vel,t=min_time,r=force,w=True)
        return self.socket_send(prog)

    def getl(self):
        """
        get TCP position
        """
        prog = self.format_prog(10)
        return self.decode_msg(prog)

    def getj(self):
        """
        get joints position
        """
        prog = self.format_prog(11)
        return self.decode_msg(prog)

    def get_inverse_kin(self,pose):
        """
        get inverse kin of pose
        """
        prog = self.format_prog(12,pose=pose)
        return self.decode_msg(prog)

    def get_forces(self):
        """
        get x,y,z forces and rx,ry,rz torques
        """
        prog = self.format_prog(14)
        return self.decode_msg(prog)

    def get_force(self):
        """
        get force magnitude
        """
        prog = self.format_prog(15)
        return float(self.socket_send(prog))

    def getlv(self):
        """
        get tool velocity
        """
        prog = self.format_prog(16)
        return self.decode_msg(prog)

    def getjv(self):
        """
        get joint velocity
        """
        prog = self.format_prog(17)
        return self.decode_msg(prog)

    def set_tcp(self, tcp):
        """
        set robot tool centre point
        """
        self.tcp = tcp
        prog = self.format_prog(20,pose=tcp)
        return self.socket_send(prog)

    def set_payload(self, weight, cog=None):
        """
        set payload in Kg
        cog is a vector x,y,z
        if cog is not specified, then tool center point is used
        """
        if cog==None:
            prog = self.format_prog(21,pose=self.tcp,acc=weight)
        else:
            prog = self.format_prog(21,pose=cog.extend([0,0,0]),acc=weight)
        return self.socket_send(prog)

    def set_digital_out(self, port, val=True):
        """
        set configurable digital out (port) to val
        """
        prog = self.format_prog(22,acc=port,vel=val)
        return self.socket_send(prog)


    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #
    #                                                                     Gripper Commands
    #
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # CMD   Description
    # C     close gripper, larger var waits longer before timing out
    # O     open gripper, larger var waits longer before timing out
    # R     rotate gripper cw, var = no. rotations
    # U     rotate gripper ccw, var = no. rotations
    # G     calibrate finger rotation and bearing position
    # F     calibrate finger rotation
    # B     calibrate bearing position
    # H     switch electromagnet to hold
    # R     release electromagnet to drop objects    

    def wait_for_gripper(self):
        """
        wait for current gripper processes to finish
        """
        self.serial_send("W",0,True)
        return

    def close_gripper(self,var=0,wait=True):
        """
        close gripper, times out after ~var seconds
        """
        self.serial_send("C",var,wait)
        return

    def open_gripper(self,var=0,wait=True):
        """
        open gripper, times out after ~5*var seconds, if var>=5 calibrate open position instead
        """
        if var>=5 and self.side=='right':
            self.serial_send("B",0,wait)
        else:
            self.serial_send("O",var,wait)
        return

    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #
    #                                                                      Special Functions
    #
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------

    #Need to write functions that converts between ROS and Robot Coordinates

    def ros_to_robot_pose_transform_no_rotation(self, pose):
        #Convert from ROS pose to Robot pose
        robot_x = pose.position.y
        robot_y = -pose.position.x
        robot_z = pose.position.z
    
        #default pointing down orientation [2.237, -2.1658,0.0] #taken from simons work
        #Default pointing down orientation - as figured out using the UR robot 
        euler = R.from_euler("xyz", (3.14149633, 0, -1.57079633))
        rotvec = euler.as_rotvec()
        rx = rotvec[0]
        ry = rotvec[1]
        rz = rotvec[2]

        robot_pose = [robot_x,robot_y,robot_z, rx, ry, rz]
        return robot_pose


    def ros_to_robot_pose_transform_at_angle(self, pose, pre_grasp = True):
        #Convert from ROS pose to Robot pose
        robot_x = pose.position.y
        robot_y = -pose.position.x
        robot_z = pose.position.z
    
        #rviz to robot configuration
        rpitch = R.from_euler('y', -90, degrees = True)
        rroll = R.from_euler('z', -90, degrees = True)
        RTrv = rpitch*rroll
        #For turning the robot on its side
        ori = [0, np.pi/2, np.radians(30)] #Desired home position (in lh RVIZ)
        ori_rh = [ori[1], ori[0], ori[2]] #now in rh position
        ori_rh = R.from_euler("xyz", ori_rh)

        rotation_in_robot = RTrv* ori_rh
        rotation_in_robot = rotation_in_robot.as_euler("xyz")
        euler = R.from_euler("xyz", rotation_in_robot)
        rotvec = euler.as_rotvec()
        rx = rotvec[0]
        ry = rotvec[1]
        rz = rotvec[2]

        '''
        if pre_grasp:
            dy = 0.4 * np.tan(np.pi/4)
        else:
            dy = 0.275 * np.tan(np.pi/4)
        '''
        
        robot_pose = [robot_x ,robot_y,robot_z, rx, ry, rz]
        return robot_pose


    def ros_to_robot_joint_pose_transform_rotate(self, pose, rotate):
        #Convert from ROS pose to Robot pose but with rotation added to the default pose
        robot_x = pose.position.y
        robot_y = -pose.position.x
        robot_z = pose.position.z
        default_orientation = [3.14149633, 0, -1.57079633]
        euler = R.from_euler("xyz", default_orientation)
        rotvec = euler.as_rotvec()
        rx = rotvec[0]
        ry = rotvec[1]
        rz = rotvec[2]

        robot_pose = [robot_x,robot_y,robot_z, rx, ry, rz]
        joint_poses = self.get_inverse_kin(robot_pose)
        joint_poses[5] = joint_poses[5] + rotate*np.pi/180.0
        
        return joint_poses




    #I dont think this is used
    def robot_to_ros_pose_transform(self, robot_pose):
        ROS_pose = Pose()
        ROS_pose.position.x = -robot_pose[1]
        ROS_pose.position.y = robot_pose[0]
        ROS_pose.position.z = robot_pose[2]

        robot_orientation = [robot_pose[3], robot_pose[4], robot_pose[5]]
        #print("original axis euler pose", robot_orientation)

        #rotates to ROS coordinates before being transformed
        quat = quaternion_from_euler(robot_orientation[0], robot_orientation[2], robot_orientation[1])

        ROS_pose.orientation.x = quat[0]
        ROS_pose.orientation.y = quat[1]
        ROS_pose.orientation.z = quat[2]
        ROS_pose.orientation.w = quat[3]
        euler = euler_from_quaternion((quat[0], quat[1], quat[2],quat[3]))
        euler_copy = list(euler)
        euler_copy[0] = euler[0]
        euler_copy[1] = euler[2]
        euler_copy[2] = euler[1]
        
        #rotates to Robot coordinates after transformed
        #print("axis to quat back to euler", euler_copy)

        return ROS_pose

