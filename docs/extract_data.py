import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

cmd_vel = pd.read_csv('cmd_vel.txt')
imu_data = pd.read_csv('imu_data.txt')
velocities = pd.read_csv('velocities.txt')
velocities_raw = pd.read_csv('velocities_raw.txt')

#ax = cmd_vel.plot(x ='%time', y='field.linear.x', kind = 'line', title='linear')	
#velocities.plot(x ='%time', y='field.linear.x', kind = 'line', ax = ax)	
#ax.set_ylim(-3,3)
#
#ax = velocities.plot(x ='%time', y='field.angular.z', kind = 'line', title='angular')	
#cmd_vel.plot(x ='%time', y='field.angular.z', kind = 'line', ax = ax)	
#ax.set_ylim(-3,3)

ax = velocities_raw.plot(x ='%time', y='field.data0', kind = 'line', title='left/right wheel velocity')	
velocities_raw.plot(x ='%time', y='field.data1', kind = 'line', ax = ax)	

ax = imu_data.plot(x ='%time', y='field.linear_acceleration.x', kind = 'line', title='linear acceleration')	
imu_data.plot(x ='%time', y='field.linear_acceleration.y', kind = 'line', ax = ax)	
imu_data.plot(x ='%time', y='field.linear_acceleration.z', kind = 'line', ax = ax)	

ax = imu_data.plot(x ='%time', y='field.angular_velocity.x', kind = 'line', title='angular_velocity')	
imu_data.plot(x ='%time', y='field.angular_velocity.y', kind = 'line', ax = ax)	
imu_data.plot(x ='%time', y='field.angular_velocity.z', kind = 'line', ax = ax)	
