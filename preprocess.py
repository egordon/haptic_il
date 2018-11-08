#!/usr/bin/python3
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter
import sys
import os
import yaml
import errno

# Get current script directory
script_dir = sys.path[0]

# Make folder if not exist
def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc:  # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise

# Read Config File
def getConfig(fName):
    try:
        with open(fName, 'r') as ymlfile:
            return yaml.load(ymlfile)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        print("Config file could not be read...")
        return None

def sortByTime(data):
    """
    Make sure that data is sorted by time column
    """
    print("Sorting by time...")
    return data[data[:,0].argsort()]

def bias_cols(data, cols, samples):
    # Get mean for each column
    means = data[:samples, cols]
    means = np.mean(means, axis=0)

    dataCp = np.copy(data)
    dataCp[:, cols] = data[:, cols] - means

    return dataCp


def bias(data, config):
    """
    Bias certain data columns based on the mean of the first few samples
    return: a copy of data that has been biased
    """

    # Get columns to bias
    cols = []
    if config["bias"]["force"]:
        cols.extend([1, 2, 3])
    if config["bias"]["torque"]:
        cols.extend([4, 5, 6])
    if len(cols) == 0:
        # Nothing to bias
        return
    cols = np.array(cols)

    # Execute biasing
    dataCp = bias_cols(data, cols, config["bias"]["samples"])

    if config["debug"]:
        # Plot biasing
        time = data[:, 0]
        for i in range(3):
            plt.subplot(1, 3, i+1)
            force_raw = data[:, i+1]
            force_bias = dataCp[:, i+1]
            plt.plot(time, force_raw, "r-", label="Raw")
            plt.plot(time, force_bias, "b-", label="Biased")
            plt.xlabel("Time (s)")
            plt.ylabel("Force (N)")
            plt.legend()
        plt.show()

    return dataCp

def pretruncate(data, config):
    """
    Makes a copy of data
    Smooths Z-force w/ savgol
    Determines "contact" when derivative of Z-force exceeds threshold
    Truncates all data to "contact"
    returns truncated data
    """

    # Get Z force
    dataZ = np.copy(data[:, 3])

    # Smooth and find gradient
    dataZ_smoothed = savgol_filter(dataZ, config["pretruncate"]["savgol_window"], config["pretruncate"]["savgol_order"])
    dataZ_grad = np.gradient(dataZ_smoothed)

    # Get X force
    dataX = np.copy(data[:, 1])

    # Smooth and find gradient
    dataX_smoothed = savgol_filter(dataX, config["pretruncate"]["savgol_window"], config["pretruncate"]["savgol_order"])
    dataX_grad = np.gradient(dataX_smoothed)

    # Look for when gradient first goes above threshold
    ind_start = np.argmax(np.logical_or(dataZ_grad > config["pretruncate"]["deriv_thresh"], dataX_grad > config["pretruncate"]["deriv_thresh"]))
    ind_start -= config["pretruncate"]["buffer"]

    # Copy Truncated Data Over
    dataCp = np.copy(data[ind_start:, :])

    if config["debug"]:
        # Plot filtering
        time = data[:, 0]
        plt.subplot(1, 2, 1)
        plt.plot(time, dataX_smoothed, "r-", label="X")
        plt.plot(time, dataZ_smoothed, "b-", label="Z")
        plt.xlabel("Time (s)")
        plt.ylabel("Z Force (N)")
        plt.legend()
        plt.subplot(1, 2, 2)
        boundary = config["pretruncate"]["deriv_thresh"] * np.ones(dataZ_grad.shape)
        plt.plot(time, dataZ_grad, "b-", label="GradientZ")
        plt.plot(time, dataX_grad, "r-", label="GradientX")
        plt.plot(time, boundary, "b-", label="Threshold")
        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Gradient (N/step)")
        plt.show()

    return dataCp

def transform(data, config):
    """
    Transforms all position data to the front of the fork.
    Moves the front of the fork into "food space", where origin is defined as the point of contact.
    Point of contact as determined in pre-truncate
    Finally, post-truncates based on height of front of fork.
    """

    # Read transform from config
    tFront = np.array(config["transform"]["fork_front"]).reshape((3, 1))

    # Create nx3x3 rotation matrix vector from Euler Angles
    Cx = np.cos(data[:, 10])
    Sx = np.sin(data[:, 10])
    Cy = np.cos(data[:, 11])
    Sy = np.sin(data[:, 11])
    Cz = np.cos(data[:, 12])
    Sz = np.sin(data[:, 12])

    # 3x3xn, need to transpose
    R = np.array([[Cz*Cy, Cz*Sy*Sx - Cx*Sz, Sz*Sx + Cz*Cx*Sy],
        [Cy*Sz, Cz*Cx + Sz*Sy*Sx, Cx*Sz*Sy - Cz*Sx],
        [-Sy, Cy*Sx, Cy*Cx]])
    R = np.transpose(R, (2, 0, 1))

    # Move to front of fork
    dataFront = np.copy(data)
    tFront = np.matmul(R, tFront).reshape(dataFront[:, 7:10].shape)
    
    dataFront[:, 7:10] += tFront

    if config["debug"]:
        # Plot transformed
        time = data[:, 0]
        back = data[:, 7:10]
        front = dataFront[:, 7:10]
        plt.plot(time, back[:, 1], "r-", label="Back Y")
        plt.plot(time, front[:, 1], "b-", label="Front Y")
        plt.xlabel("Time (s)")
        plt.ylabel("Y Pose (m)")
        plt.legend()
        plt.show()

    # Bias position to contact point
    cols = np.array([7, 8, 9])
    dataFront = bias_cols(dataFront, cols, config["transform"]["samples"])

    # Post-Truncate to when front of fork is above contact point
    # by some threshold
    ind_end = np.argmax(dataFront[:, 8] > config["posttruncate"]["y_thresh"])
    ind_end += config["posttruncate"]["buffer"]
    dataFront = dataFront[:ind_end, :]

    if config["debug"]:
        # Plot transformed
        time = dataFront[:, 0]
        plt.title("Front Relative to Contact")
        plt.plot(time, dataFront[:, 7], "r-", label="X")
        plt.plot(time, dataFront[:, 8], "g-", label="Y")
        plt.plot(time, dataFront[:, 9], "b-", label="Z")
        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Pose (m)")
        plt.show()

    return dataFront

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: preprocess.py <config_file> [folder = raw]")
        print("Note: all files and folders relative to script dir unless absolute path given")
        sys.exit(0)

    # Select data folder
    folder = "raw"
    if len(sys.argv) > 2:
        folder = sys.argv[2]

    # Check data dir
    if folder[0] == '/':
        data_dir = folder
    else:
        data_dir = script_dir + "/" + folder
    if not os.path.isdir(data_dir):
        print("Error: data not in folder (" + data_dir + "), please run download_data.py")
        sys.exit(-1)

    preprocessed_dir = script_dir + "/preprocessed"

    # Get Config File
    config = getConfig(script_dir + "/" + sys.argv[1])
    if config is None:
        sys.exit(-1)

    # Open File
    obs = []
    acs = []
    for sNum in range(1, 13):
        for tNum in range(1, 21):
            subDir = "/subject" + str(sNum) + "_potato_salad/"
            fEnd = str(tNum) + ".csv"
            fName = data_dir + subDir + fEnd
            writeName = preprocessed_dir + subDir + fEnd
            print("Opening CSV file: " + fName)
            try:
                processed = None
                dataHeader = None
                data = None
                with open(fName, 'r') as f:
                    dataHeader = f.readline()
                    data = np.loadtxt(f, delimiter=',', skiprows=0)

                # Bias Data
                processed = bias(data, config)

                # Pre-Truncate Data
                processed = pretruncate(processed, config)

                # Transfom to food frame and post-truncate
                processed = transform(processed, config)

                # Write post-processed data to CSV file
                mkdir_p(preprocessed_dir + subDir)
                print("Writing CSV file: " + writeName)
                with open(writeName, 'w+') as wf:
                    wf.write(dataHeader + "\n")
                    np.savetxt(wf, processed, delimiter=",")

                # Append to episodes lists
                obs.append(processed[:, config["npz"]["obs_cols"]])
                acs.append(processed[:, config["npz"]["acs_cols"]])

            except EnvironmentError as e: # parent of IOError, OSError *and* WindowsError where available
                print("Data file could not be read: (" + fName + ")")
                print("More Data: ")
                print(e)

    # Pad each array in list to max rows
    # Pad by repeating last row
    newrows = np.max([i.shape[0] for i in obs])
    for i in range(len(obs)):
        obs[i] = np.pad(obs[i], ((0, newrows-obs[i].shape[0]),(0,0)), 'edge')

    newrows = np.max([i.shape[0] for i in acs])
    for i in range(len(acs)):
        acs[i] = np.pad(acs[i], ((0, newrows-acs[i].shape[0]),(0,0)), 'edge')

    # Concat into 3D array and save
    obs = np.dstack(obs)
    obs = np.transpose(obs, (2, 0, 1))

    print("Writing observations with shape: " + str(obs.shape))

    acs = np.dstack(acs)
    acs = np.transpose(acs, (2, 0, 1))

    print("Writing actions with shape: " + str(acs.shape))

    writeName = preprocessed_dir + "/trajectories_compressed.npz"
    print("Writing to file: " + writeName)
    np.savez_compressed(writeName, obs=obs, acs=acs)