#!/usr/bin/python3
import matplotlib.pyplot as plt
import numpy as np
import sys

# Get current script directory
script_dir = sys.path[0]

def plotFile(f):
    data = np.loadtxt(f, delimiter=',', skiprows=1)

    # Sort by time column
    print("Sorting by time...")
    data = data[data[:,0].argsort()]

    # Plot Position and Force Data
    time = data[:, 0]
    force = data[:, 1:4]
    pose = data[:, 7:10]
    torque = data[:, 4:7]

    plt.subplot(221)
    forceStrings = ["Force X", "Force Y", "Force Z"]
    for i in range(3):
        plt.plot(time, force[:, i], label=forceStrings[i])

    plt.xlabel("Time (s)")
    plt.ylabel("Force (N)")
    plt.legend()

    plt.subplot(222)
    poseStrings = ["Pose X", "Pose Y", "Pose Z"]
    for i in range(3):
        plt.plot(time, pose[:, i], label=poseStrings[i])

    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.legend()

    plt.subplot(223)
    torqueStrings = ["Torque X", "Torque Y", "Torque Z"]
    for i in range(3):
        plt.plot(time, torque[:, i], label=torqueStrings[i])

    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.legend()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: plot_csv.py <subject_num> <trial_num> [folder=raw]")
        sys.exit(0)

    # Record system and trial number
    sNum = sys.argv[1]
    tNum = sys.argv[2]

    # Select data folder
    folder = "raw"
    if len(sys.argv) > 3:
        folder = sys.argv[3]

    # Open File
    fName = script_dir + "/" + str(folder) + "/subject" + str(sNum) + "_potato_salad/" + str(tNum) + ".csv"
    print("Opening CSV file: " + fName)
    with open(fName) as f:
        plt.figure(1)
        plt.title("Subject " + str(sNum) + "; Trial: " + str(tNum))
        plotFile(f)
        plt.show()