#!/usr/bin/python3

import numpy as np
import os, sys
import readline
import math
import matplotlib.pyplot as plt
from scipy.spatial.distance import euclidean, directed_hausdorff
from dtw import dtw # https://dynamictimewarping.github.io/python/ pip3/pip install dtw-python

readline.set_completer_delims(' \t\n=')
readline.parse_and_bind('tab: complete')

# Get full directory to file to open if not specified in command line
if(len(sys.argv) < 2):
    log_file = input("Full path directory to log file including extension: ")
else:
    log_file = sys.argv[1]

log_file = os.path.expanduser(log_file)

# Exclude empty input, analyse from the back
full_length = 0
try:
    dummy_file = open(str(log_file), 'r')
except:
    print("Unable to open file, it either does not exist or there are permission errors")
    exit()

dummy_file.readline()
dummy_user_input = np.empty((0,2), float)
while 1:
    line = dummy_file.readline()
    if len(line) == 0:
        break
    
    # Split line without including newline character \n
    split_string = line[0: len(line) - 1].split(",")

    # Add to user_input
    new_user_input = np.array([[float(split_string[15]), float(split_string[16])]])
    if not (math.isnan(new_user_input[0][0]) and math.isnan(new_user_input[0][1])):
        dummy_user_input = np.append(dummy_user_input, new_user_input, axis=0)

for i in reversed(range(len(dummy_user_input))):
    if (dummy_user_input[i][0] != 0.0) or (dummy_user_input[i][1] != 0.0):
        full_length = i
        break

# Open file, exit if failed
try:
    f = open(str(log_file), "r")
except:
    print("Unable to open file, it either does not exist or there are permission errors")
    exit()

# Read until end and convert each value into numbers
# Paths are stored as np.arrays of arrays, each coordinate is stored in an array of 2 elements
ref_path = np.empty((0,2), float)
user_path = np.empty((0,2), float)
user_time = np.array([])
user_input = np.empty((0,2), float)
robot_velocity = np.empty((0,2), float)

# Read away the header line
f.readline()

# ref.x,ref.y,ref.z,ref.o.x,ref.o.y,ref.o.z,ref.o.w,user.time,user.x,user.y,user.z,user.o.x,user.o.y,user.o.z,user.o.w,user.linvel,user.angvel,robot.linvel,robot.angvel
while 1:
    line = f.readline()
    if len(line) == 0:
        break
    
    # Split line without including newline character \n
    split_string = line[0: len(line) - 1].split(",")

    # Add to ref_path
    new_ref_pose = np.array([[float(split_string[0]), float(split_string[1])]])
    if not (math.isnan(new_ref_pose[0][0]) and math.isnan(new_ref_pose[0][1])):
        ref_path = np.append(ref_path, new_ref_pose, axis=0)

    if len(user_path) < full_length:
        # Add to user_path
        new_user_pose = np.array([[float(split_string[8]), float(split_string[9])]])
        if not (math.isnan(new_user_pose[0][0]) and math.isnan(new_user_pose[0][1])):
            user_path = np.append(user_path, new_user_pose, axis=0)

        # Add to user_input
        new_user_input = np.array([[float(split_string[15]), float(split_string[16])]])
        if not (math.isnan(new_user_input[0][0]) and math.isnan(new_user_input[0][1])):
            user_input = np.append(user_input, new_user_input, axis=0)

        # Add to user_time
        if not (math.isnan(float(split_string[7]))):
            user_time = np.append(user_time, float(split_string[7]))

        # Add to robot_velocity
        new_robot_vel = np.array([[float(split_string[17]), float (split_string[18])]])
        if not (math.isnan(new_robot_vel[0][0]) and math.isnan(new_robot_vel[0][1])):
            robot_velocity = np.append(robot_velocity, new_robot_vel, axis=0)

# Plot user path and reference path
user_path_x, user_path_y = zip(*user_path)
ref_path_x, ref_path_y = zip(*ref_path)
path_plot = plt.figure(1)
plt.plot(user_path_x, user_path_y, color='darkorange', label="User's path")
plt.plot(ref_path_x, ref_path_y, "--", color='black', label="Reference path")
if(len(sys.argv) > 2):
    if (sys.argv[2] == 'D'):
        door1 = plt.Rectangle((3.0, 0.4),0.2,1.0,edgecolor='r')
        door2 = plt.Rectangle((3.0, -1.62),0.2,1.0,edgecolor='r')
        door3 = plt.Rectangle((5.66, -0.5),0.2,1.0,edgecolor='r')
        door4 = plt.Rectangle((9.02, 0.17),0.2,1.0,edgecolor='r')
        door5 = plt.Rectangle((9.02, -1.25),0.2,1.0,edgecolor='r')
        door6 = plt.Rectangle((11.7, 0.85),0.2,1.0,edgecolor='r')
        door7 = plt.Rectangle((11.7, -0.8),0.2,1.0,edgecolor='r')
        door8 = plt.Rectangle((11.7, -2.6),0.2,1.0,edgecolor='r')
        for i in range(1,9):
            door_idx = str(i)
            eval('plt.gca().add_patch(door'+ door_idx +')')
    elif (sys.argv[2] == 'H'):
        door1 = plt.Rectangle((2.52, 1.7),1.2,0.2,edgecolor='r')
        door2 = plt.Rectangle((4.4, 1.7),1.2,0.2,edgecolor='r')
        door3 = plt.Rectangle((11.9, 0.3),0.2,1.2,edgecolor='r')
        door4 = plt.Rectangle((11.9, 5.2),0.2,1.2,edgecolor='r')
        door5 = plt.Rectangle((6.0, 13.15),0.2,1.2,edgecolor='r')
        for i in range(1,6):
            door_idx = str(i)
            eval('plt.gca().add_patch(door'+ door_idx +')')
plt.title("Paths")
plt.xlabel("x-coordinate")
plt.ylabel("y-coordinate")
plt.legend()
plt.grid(True)
plt.axis('scaled')
# eval('plt.savefig(\'SF-'+ log_file[-9:-4] +'.png\')')

# Get average robot linear and angular acceleration
robot_acc = np.array([])
robot_ang_acc = np.array([])
for i in range(len(robot_velocity) - 1):
    dvx = robot_velocity[i+1][0] - robot_velocity[i][0]
    dwz = robot_velocity[i+1][1] - robot_velocity[i][1]
    dt = user_time[i+1] - user_time[i]
    if(dt == 0):
        robot_acc = np.append(robot_acc, 0)
        robot_ang_acc = np.append(robot_ang_acc, 0)
    else:
        robot_acc = np.append(robot_acc, dvx/dt)
        robot_ang_acc = np.append(robot_ang_acc, dwz/dt)

# Plot robot's acceleration and print average acceleration
robot_acc_plot = plt.figure(2)
plt.plot(user_time[0: len(user_time) - 1], robot_acc, label="Linear acceleration", ls="-.")
plt.plot(user_time[0: len(user_time) - 1], robot_ang_acc, label="Angular acceleration", ls="--", alpha=0.5)
plt.title("Robot Acceleration")
plt.xlabel("Time")
plt.ylabel("Acceleration (m/s^2) / (rad/s^2)")
plt.legend()
plt.grid(True)

# Get average user linear and angular acceleration
user_acc = np.array([])
user_ang_acc = np.array([])
for i in range(len(user_input) - 1):
    dvx = user_input[i+1][0] - user_input[i][0]
    dwz = user_input[i+1][1] - user_input[i][1]
    dt = user_time[i+1] - user_time[i]
    if(dt == 0):
        user_acc = np.append(user_acc, 0)
        user_ang_acc = np.append(user_ang_acc, 0)
    else:
        user_acc = np.append(user_acc, dvx/dt)
        user_ang_acc = np.append(user_ang_acc, dwz/dt)

# Plot robot's acceleration and print average acceleration
user_acc_plot = plt.figure(3)
plt.plot(user_time[0: len(user_time) - 1], user_acc, label="Linear Input Rate of Change", ls="-.")
plt.plot(user_time[0: len(user_time) - 1], user_ang_acc, label="Angular Input Rate of Change", ls="--", alpha=0.5)
plt.title("User Input Rate of Change")
plt.xlabel("Time")
plt.ylabel("Rate of change (m/s^2) / (rad/s^2)")
plt.legend()
plt.grid(True)

# Compute deviation from reference path using different metrics
normal_dtw = -1
try:
    normal_dtw = dtw(ref_path, user_path, distance_only=True).normalizedDistance
except:
    print("Normalized dtw was unable to be calculated")
hausdorff_dist = directed_hausdorff(ref_path, user_path)[0]

# Calculate path smoothness with linear and angular acceleration squared averages
avg_lin_acc = np.average(np.square(robot_acc))
avg_ang_acc = np.average(np.square(robot_ang_acc))

# Calculate user burden based on user input's squared averages
avg_user_lin_acc = np.average(np.square(user_acc))
avg_user_ang_acc = np.average(np.square(user_ang_acc))

# Output all results
print("Normalized dtw distance: " + str(normal_dtw))
print("Directed Hausdorff distance: " + str(hausdorff_dist))
print("Robot squared average linear acceleration: " + str(avg_lin_acc) + " (m/s^2)^2")
print("Robot squared average angular acceleration: " + str(avg_ang_acc) + " (rad/s^2)^2")
print("User input squared average linear acceleration: " + str(avg_user_lin_acc) + " (m/s^2)^2")
print("User input squared average angular acceleration: " + str(avg_user_ang_acc) + " (rad/s^2)^2")
print("Total time of user's trajectory: " + str(user_time[-1]) + "s")

# Save computed results into output file
save_file = log_file[0: len(log_file) - 4] + "_results.txt"
save = open(save_file, "w")
save.writelines("normalized_dtw,directed_hausdorff,robot_avg_sqrd_lin_acc,robot_avg_sqrd_ang_acc,user_avg_sqrd_lin_acc,user_avg_sqrd_ang_acc,total_time\n")
save.writelines(str(normal_dtw) + "," + str(hausdorff_dist) + "," + str(avg_lin_acc) + "," + str(avg_ang_acc) + "," + str(avg_user_lin_acc) + "," + str(avg_user_ang_acc) + "," + str(user_time[-1]))

save.close()
f.close()

if(len(sys.argv) > 2):
    plt.show()
