#!/usr/bin/python3

import numpy as np
import os, sys
import readline
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
import matplotlib.pylab as pl
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
dummy_joystick = np.empty((0,2), float)
while 1:
    line = dummy_file.readline()
    if len(line) == 0:
        break
    
    # Split line without including newline character \n
    split_string = line[0: len(line) - 1].split(",")

    # Add to user_input
    new_joystick = np.array([[float(split_string[19]), float(split_string[20])]])
    if not (math.isnan(new_joystick[0][0]) and math.isnan(new_joystick[0][1])):
        dummy_joystick = np.append(dummy_joystick, new_joystick, axis=0)

for i in reversed(range(len(dummy_joystick))):
    if (abs(dummy_joystick[i][0]) >= 0.1) or (abs(dummy_joystick[i][1]) >= 0.1):
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
# user_yaw = np.array([])
user_heading = np.empty((0,2), float)
user_time = np.array([])
cmd_input = np.empty((0,2), float)
joystick_input = np.empty((0,2), float)
robot_velocity = np.empty((0,2), float)
user_weight = np.array([])
path_index = np.array([])
angle_diff = np.array([])
path_color = np.empty((0,3), float)

# Read away the header line
f.readline()

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

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

        # Add to user_heading
        if not (math.isnan(float(split_string[11])) and math.isnan(float(split_string[12])) and math.isnan(float(split_string[13])) and math.isnan(float(split_string[14]))):
            [new_roll, new_pitch, new_yaw] = euler_from_quaternion(float(split_string[11]), float(split_string[12]), float(split_string[13]), float(split_string[14]))
            # user_yaw = np.append(user_yaw, new_yaw)

            v = np.array([0.5, 0])
            rot = np.array([[math.cos(new_yaw), -math.sin(new_yaw)], [math.sin(new_yaw), math.cos(new_yaw)]])
            v2 = np.dot(rot, v)
            # new_user_heading = np.array([[new_user_pose[0] + v2[0], new_user_pose[1] + v2[1]]])
            new_user_heading = np.array([[float(split_string[8]) + v2[0], float(split_string[9]) + v2[1]]])
            user_heading = np.append(user_heading, new_user_heading, axis=0)

        # Add to cmd_input
        new_cmd_input = np.array([[float(split_string[15]), float(split_string[16])]])
        if not (math.isnan(new_cmd_input[0][0]) and math.isnan(new_cmd_input[0][1])):
            cmd_input = np.append(cmd_input, new_cmd_input, axis=0)

        # Add to user_time
        if not (math.isnan(float(split_string[7]))):
            user_time = np.append(user_time, float(split_string[7]))

        # Add to robot_velocity
        new_robot_vel = np.array([[float(split_string[17]), float (split_string[18])]])
        if not (math.isnan(new_robot_vel[0][0]) and math.isnan(new_robot_vel[0][1])):
            robot_velocity = np.append(robot_velocity, new_robot_vel, axis=0)
        
        # Add to joystick_input
        new_joystick_input = np.array([[float(split_string[19]), float (split_string[20])]])
        if not (math.isnan(new_joystick_input[0][0]) and math.isnan(new_joystick_input[0][1])):
            joystick_input = np.append(joystick_input, new_joystick_input, axis=0)

        # Add to user_weight
        if not (math.isnan(float(split_string[21]))):
            user_weight = np.append(user_weight, float(split_string[21]))

        # Add to path_id
        if not (math.isnan(float(split_string[23]))):
            path_index = np.append(path_index, float(split_string[23]))

# Plot user path and reference path
user_path_x, user_path_y = zip(*user_path)
user_heading_x, user_heading_y = zip(*user_heading)
ref_path_x, ref_path_y = zip(*ref_path)
joystick_input_x, joystick_input_y = zip(*joystick_input)

path_id = []
num_index = 0

# print(user_heading_y)

# print(np.max(user_yaw))
# print(np.min(user_yaw))
# for i in range(len(user_path)):
#     if ((cmd_input[i][0] == 0 and cmd_input[i][1] == 0) or (joystick_input[i][0] == 0 and joystick_input[i][1] == 0)):
#         angle_diff = np.append(angle_diff, 0.0)
#     else:
#         unit_vector_1 = cmd_input[i] / np.linalg.norm(cmd_input[i])
#         unit_vector_2 = joystick_input[i] / np.linalg.norm(joystick_input[i])
#         dot_product = np.dot(unit_vector_1, unit_vector_2)
#         dot_product = round(dot_product, 5)
#         angle_vector = np.arccos(dot_product)
#         angle_diff = np.append(angle_diff, angle_vector)
#     if (path_index[i] == 4 or path_index[i] == 1760):
#         path_color = np.append(path_color, np.array([[205/255, 96/255, 144/255]]), axis=0)
#     elif (path_index[i] == 1):
#         path_color = np.append(path_color, np.array([[0, 0, 238/255]]), axis=0)
#     elif (path_index[i] == 3):
#         path_color = np.append(path_color, np.array([[178/255, 58/255,238/255]]), axis=0)
#     # if (path_index[i] == num_index):
#     #     continue
#     # else:
#     #     num_index = path_index[i]
#     #     if (path_index[i] not in path_id):
#     #         path_id.append(path_index[i])

# # for i in range(len(user_path)):
# #     color_idx = path_id.index(path_index[i]) * 0.333333
# #     path_color = np.append(path_color, color_idx)
# # print(np.amax(angle_diff)*180/np.pi)
# max_angle_diff = np.amax(angle_diff)
# norm_angle_diff = angle_diff / max_angle_diff
# max_user_weight = np.amax(user_weight)
# user_weight = user_weight / max_user_weight
# print(np.amin(max_user_weight))
path_plot = plt.figure(1)
# rgb_angle = pl.cm.cool(norm_angle_diff)
rgb_weight = pl.cm.cool(user_weight)
# rgb_path = pl.cm.cool(path_color)
# im = plt.imread("/home/ray/hospital_cut_path.png")
# plt.imshow(im, extent=[-1.55, 15.10, -1.45, 15.2])
# plt.plot(user_path_x, user_path_y, color='darkorange', label="User's path")
# quiver is not applicable here because the path points are calculated in the map frame but joystick input data was recorded in the local frame
# plt.quiver(user_path_x, user_path_y, joystick_input_x, joystick_input_y, angles = "xy", color = 'r', pivot = "mid", scale_units = 'xy', scale = 4)
plt.scatter(user_path_x, user_path_y, color='orange', marker='.', linewidths = 0.2, label="User's path")
plt.plot(ref_path_x, ref_path_y, "--", color='black', label="Reference path")
if(len(sys.argv) > 2):
    if (sys.argv[2] == 'D'):
        door1 = plt.Rectangle((-0.5, -2.7),1.5,0.2,edgecolor='r')
        door2 = plt.Rectangle((2.0, -2.7),1.5,0.2,edgecolor='r')
        door3 = plt.Rectangle((5.0, -2.7),1.5,0.2,edgecolor='r')
        door4 = plt.Rectangle((0.95, -6.1),1.5,0.2,edgecolor='r')
        door5 = plt.Rectangle((4.1, -6.1),1.5,0.2,edgecolor='r')
        door6 = plt.Rectangle((2.45, -9.6),1.5,0.2,edgecolor='r')
        door7 = plt.Rectangle((0.95, -13.54),1.5,0.2,edgecolor='r')
        door8 = plt.Rectangle((4.1, -13.54),1.5,0.2,edgecolor='r')
        for i in range(1,9):
            door_idx = str(i)
            eval('plt.gca().add_patch(door'+ door_idx +')')
    elif (sys.argv[2] == 'H'):
        door1 = plt.Rectangle((2.39, 1.7),1.5,0.2,edgecolor='r')
        door2 = plt.Rectangle((4.25, 1.7),1.5,0.2,edgecolor='r')
        door3 = plt.Rectangle((11.9, 0.2),0.2,1.5,edgecolor='r')
        door4 = plt.Rectangle((11.9, 5.05),0.2,1.5,edgecolor='r')
        door5 = plt.Rectangle((5.8, 13.05),0.2,1.5,edgecolor='r')
        for i in range(1,6):
            door_idx = str(i)
            eval('plt.gca().add_patch(door'+ door_idx +')')
# plt.title("Change of user control weight during the navigation")
# plt.title("Change of angle difference between original joystick input and executed action")
plt.title("Change of intention prediction during the navigation")
# plt.title("CP Subject 2's trajectory using shared-DWA")
# plt.title("CP Subject 2's trajectory using the proposed method")
plt.xlabel("x-coordinate")
plt.ylabel("y-coordinate")
plt.legend()
# plt.text()
# plt.grid(True)
# plt.axis([-1, 15, -1, 15])
plt.axis('scaled')
# plt.xlim((-1, 15))
# plt.ylim((-1, 15))
# eval('plt.savefig(\'SF-'+ log_file[-9:-4] +'.png\')')

# def update_points(frame):
#     point_ani.set_data(user_path_x[frame], user_path_y[frame])
#     point_head.set_data(user_heading_x[frame], user_heading_y[frame])
#     text_pt.set_position((user_path_x[frame], user_path_y[frame]))
#     text_pt.set_text("time = %.3f"%(user_time[frame]))
#     return point_ani, point_head, text_pt,
# point_ani, = plt.plot(user_path_x[0], user_path_y[0], "ro")
# point_head, = plt.plot(user_heading_x[0], user_heading_y[0], "b*")
# # point_ani, = plt.arrow(user_path_x[0], user_path_y[0], 0.1, 0.1, width = 0.05)
# # point_ani, = plt.quiver(user_path_x[0], user_path_y[0], 0.1, 0.1, angles = "xy")
# text_pt = plt.text(0, 0, '', fontsize=10)
# ani = animation.FuncAnimation(path_plot, update_points, np.arange(0, len(user_path_x)), interval= 2, repeat=False, blit=True)

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
for i in range(len(joystick_input) - 1):
    dvx = joystick_input[i+1][0] - joystick_input[i][0]
    dwz = joystick_input[i+1][1] - joystick_input[i][1]
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
save.writelines("normalized_dtw, directed_hausdorff, robot_avg_sqrd_lin_acc, robot_avg_sqrd_ang_acc, user_avg_sqrd_lin_acc, user_avg_sqrd_ang_acc, total_time\n")
save.writelines(str(normal_dtw) + ", " + str(hausdorff_dist) + ", " + str(avg_lin_acc) + ", " + str(avg_ang_acc) + ", " + str(avg_user_lin_acc) + ", " + str(avg_user_ang_acc) + ", " + str(user_time[-1]))

save.close()
f.close()

if(len(sys.argv) > 2):
    plt.show()
