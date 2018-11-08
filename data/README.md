Raw Data
===

1. Data sequence

| Time(s) | F_x(N) | F_y(N) | F_z(N) | T_x(Nm) | T_y(Nm) | T_z(Nm) | P_x(m) | P_y(m) | P_z(m) | O_x(rad) | O_y(rad) | O_z(rad) | V_px(m/s) | V_py(m/s) | V_pz(m/s) | V_ox(rad/s) | V_oy(rad/s) | V_oz(rad/s) |

F_x, F_y, F_z: Forces detected based on the (left-handed) sensor coordinate.
T_x, T_y, T_z: Torques detected based on the (right-handed) sensor coordinate.
P_x, P_y, P_z: Positions of the center of the markers on the forque based on the global coordinate.
O_x, O_y, O_z: Euler angles of the forque. The initial orientation can be found from the experimental setup figure on the paper.
V_px, V_py, V_pz: Translational velocities of the forque based on the backward differentiation of the position.
O_px, O_py, O_pz: Angular velocities of the forque based on the backward differentiation of the euler angles.

2. Coordinate frame

The forces and torques are in the local coordinate frame of the forque. The poses and velocities are in the world frame. The frames are shown in the paper.

3. Rotation

The orientaion follows ZYX Euler convention.

4. Data Length

Data lasts for the entire trajectory: from the start of recording until the food is placed in front of the mannequin's mouth.

Processed Data
===

1. Data sequence

| Time(s) | F_x(N) | F_y(N) | F_z(N) | T_x(Nm) | T_y(Nm) | T_z(Nm) | P_x(m) | P_y(m) | P_z(m) | O_x(rad) | O_y(rad) | O_z(rad) | V_px(m/s) | V_py(m/s) | V_pz(m/s) | V_ox(rad/s) | V_oy(rad/s) | V_oz(rad/s) |

Food Frame: Orientation matches global frame. Origin is initial contact point between fork and food for the given trial.

F_x, F_y, F_z: Forces detected based on the (left-handed) sensor coordinate.
T_x, T_y, T_z: Torques detected based on the (right-handed) sensor coordinate.
P_x, P_y, P_z: Positions of the tip of the forque in food frame
O_x, O_y, O_z: ZYX euler angles of the forque relative to global frame
V_px, V_py, V_pz: Translational velocities of the forque based on the backward differentiation of the position.
O_px, O_py, O_pz: Angular velocities of the forque based on the backward differentiation of the euler angles.


2. Data Length

Data is truncated from within half a second of initial contact to the point where the tip of the fork is more than 1cm above initial contact.

Scripts
===

* **download_data.py**: Downloads scooping data from Harvard dataverse into "dataverse" and unpacks it into "raw"
* **plot_csv.py &lt;folder=raw|preprocessed&gt; &lt;subject_num&gt; &lt;trial_num&gt;**: Plots force and back-fork position raw data

preprocess.py &lt;config file&gt;
===

**Code Outline:**

1. Bias force data using the first few samples
2. Pre-truncate to the "contact point," where we have a significant increase in X or Z force on the fork.
3. Transform position data to front of fork.
4. Define the "contact point" as the new origin, transform position data to that new origin
5. Post-truncate when front of fork is a few cm above the contact point (positive Y)
6. Save pre-processed data to CSV files
7. Pad data to longest trajectory by repeating last datapoint, then save 3D array of trajectories (observations and actions) as npz file for use by OpenAI Baselines.