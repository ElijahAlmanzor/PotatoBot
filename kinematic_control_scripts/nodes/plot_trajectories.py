#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the directory where trajectory files are stored
TRAJ_DIR = os.path.join(os.path.expanduser('~'), 'pp_ws', 'src', 'planter', 'planter_kg_control', 'trajexp12')

def load_trajectories(num_files=5):
    """Loads trajectory data from .npy files."""
    all_traj = []
    for num in range(num_files):
        filename = f"{num}.npy"
        file_path = os.path.join(TRAJ_DIR, filename)
        try:
            with open(file_path, "rb") as f:
                traj = np.load(f)
                all_traj.append(traj)
        except FileNotFoundError:
            print(f"File not found: {filename}")
    return all_traj

def plot_trajectories(all_traj):
    """Plots TCP Cartesian coordinates over time and in 3D space."""
    fig, ax = plt.subplots(3, sharex=True)
    fig.suptitle('TCP Cartesian Coordinates (X, Y, Z) vs. Time')

    colors = ['k', 'r', 'b', 'c', 'y']
    labels = [f"Trajectory {i+1}" for i in range(len(all_traj))]

    # Plot X, Y, Z coordinates over time
    for i, traj in enumerate(all_traj):
        ax[0].plot(traj[:, 3], traj[:, 0], f'{colors[i]}--', label=labels[i])
        ax[1].plot(traj[:, 3], traj[:, 1], f'{colors[i]}--')
        ax[2].plot(traj[:, 3], traj[:, 2] - 0.4, f'{colors[i]}--')

    ax[0].set_ylabel("X (m)")
    ax[1].set_ylabel("Y (m)")
    ax[2].set_ylabel("Z (m)")
    ax[2].set_xlabel("Time (s)")
    ax[1].legend()

    # 3D Plot
    fig2 = plt.figure()
    fig2.suptitle('TCP Cartesian Coordinates (X, Y, Z)')
    ax2 = fig2.add_subplot(111, projection='3d')

    for i, traj in enumerate(all_traj):
        ax2.plot3D(traj[:, 0], traj[:, 1], traj[:, 2] - 0.4, f'{colors[i]}--', label=labels[i])

    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_zlabel("Z (m)")
    ax2.legend()

    plt.show()

def main():
    all_traj = load_trajectories()
    if all_traj:
        plot_trajectories(all_traj)
    else:
        print("No trajectory data found.")

if __name__ == '__main__':
    main()
