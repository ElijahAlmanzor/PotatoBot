#!/usr/bin/env python3
import time
import serial
from math import pi
import numpy
import socket
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#from kg_robot import kg_robot as kgr


import numpy as np
import os




def main():

    all_traj = []



    #Load in the data
    for num in range(1,25):
        try:
            name = str(num) + ".npy"
            load_path = os.path.join(os.path.expanduser('~'), 'pp_ws', 'src', 'planter', 'planter_kg_control', 'traj_angled', name)
            #load_path = os.path.join(os.path.expanduser('~'), 'pp_ws', 'src', 'planter', 'planter_kg_control', 'trajexpfigure', name)
            with open(load_path, "rb") as f:
                traj = np.load(f)
                all_traj.append(traj)
        except:
            print("doesnt exist")
        #print(traj)
    
    print(all_traj)

    fig, ax = plt.subplots(3, sharex = True)
    fig.suptitle('Angled Picking Coordinates (X,Y,Z) against Time (s)')
    fig2 = plt.figure()
    fig2.suptitle('TCP Cartesian Coordinates (X,Y,Z)')
    ax2 = fig2.add_subplot(111, projection='3d')
    for num in range(1,16):
        num -= 1
        #fisrt index is the data
        t1 = ax[0].plot(all_traj[num][:,3],  all_traj[num][:,0], c= '0.75',)
        #t2 = ax[0].plot(all_traj[1][:,3], all_traj[1][:,0], 'r--', label = "Trajectory 2")
        #t3 = ax[0].plot(all_traj[2][:,3], all_traj[2][:,0], 'b--', label = "Trajectory 3")
        #t4 = ax[0].plot(all_traj[3][:,3], all_traj[3][:,0], 'c--', label = "Trajectory 4")
        #t5 = ax[0].plot(all_traj[4][:,3], all_traj[4][:,0], 'y--', label = "Trajectory 5")
        #ax[0].set_ylabel("X coordinate (m)")

        #ax[1].plot(all_traj[num][:,3], all_traj[num][:,1],  label = "Trajectory " + str(num))
        ax[1].plot(all_traj[num][:,3], all_traj[num][:,1], c= '0.75')
        #ax[1].plot(all_traj[1][:,3], all_traj[1][:,1], 'r--', label = "Trajectory 2")
        #ax[1].plot(all_traj[2][:,3], all_traj[2][:,1], 'b--', label = "Trajectory 3")
        #ax[1].plot(all_traj[3][:,3], all_traj[3][:,1], 'c--', label = "Trajectory 4")
        #ax[1].plot(all_traj[4][:,3], all_traj[4][:,1], 'y--', label = "Trajectory 5")
        #ax[1].set_ylabel("Y coordinate (m)")

        ax[2].plot(all_traj[num][:,3], all_traj[num][:,2]- 0.4, c= '0.75')
        #ax[2].plot(all_traj[1][:,3], all_traj[1][:,2]- 0.4, 'r--')
        #ax[2].plot(all_traj[2][:,3], all_traj[2][:,2]- 0.4, 'b--')
        #ax[2].plot(all_traj[3][:,3], all_traj[3][:,2]- 0.4, 'c--')
        #ax[2].plot(all_traj[4][:,3], all_traj[4][:,2]- 0.4, 'y--')
        #ax[2].set_ylabel("Z coordinate (m)")

        #ax[2].set_xlabel("Time (s)")
        #ax[1].legend()
        
        #figure 2
        #fig2 = plt.figure()
        #fig2.suptitle('TCP Cartesian Coordinates (X,Y,Z)')
        #ax2 = fig2.add_subplot(111, projection='3d')
        ax2.plot3D(all_traj[num][:,0],all_traj[num][:,1],all_traj[num][:,2]-0.4,c= '0.75',label = "Trajectory " + str(num))
        #ax2.plot3D(all_traj[1][:,0],all_traj[1][:,1],all_traj[1][:,2]-0.4,'r--',label = "Trajectory 2")
        #ax2.plot3D(all_traj[2][:,0],all_traj[2][:,1],all_traj[2][:,2]-0.4,'b--',label = "Trajectory 3")
        #ax2.plot3D(all_traj[3][:,0],all_traj[3][:,1],all_traj[3][:,2]-0.4,label = "Trajectory 4")
        #ax2.plot3D(all_traj[4][:,0],all_traj[4][:,1],all_traj[4][:,2]-0.4,label = "Trajectory 5")
        #ax2.set_xlabel("X coordinate (m)")
        #ax2.set_ylabel("Y coordinate (m)")
        #ax2.set_zlabel("Z coordinate (m)")
        #ax2.scatter3D(potatoes_measured[:,0], potatoes_measured[:,1], potatoes_measured[:,2], 'kx')
        #ax2.legend()
    ax[0].set_ylabel("X (m)")
    ax[1].set_ylabel("Y (m)")
    ax[2].set_ylabel("Z (m)")
    ax[2].set_xlabel("Time (s)")
    

    #picking a random representative trajectory 
    rep = 7
    

    ax[0].plot(all_traj[rep-1][:,3],  all_traj[rep-1][:,0], c= '0.75', label = "Example Experimental Trajectories")
    ax[1].plot(all_traj[rep-1][:,3], all_traj[rep-1][:,1], c= '0.75')
    ax[2].plot(all_traj[rep-1][:,3], all_traj[rep-1][:,2]- 0.4, c= '0.75')
    ax[0].plot(all_traj[rep][:,3],  all_traj[rep][:,0], 'k', label = "Representative Trajectory")
    ax[1].plot(all_traj[rep][:,3], all_traj[rep][:,1], 'k')
    ax[2].plot(all_traj[rep][:,3], all_traj[rep][:,2]- 0.4, 'k')
    #ax[0].legend()
    ax2.set_xlabel("X coordinate (m)")
    ax2.set_ylabel("Y coordinate (m)")
    ax2.set_zlabel("Z coordinate (m)")
    ax2.legend()


    #mean_rep_traj = np.mean(np.asarray(all_traj), axis = 0)
    #print(mean_rep_traj)
    plt.show()
        
if __name__ == '__main__':
    main()
