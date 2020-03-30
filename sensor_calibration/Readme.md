sensor_calibration
==================

This package provides easy automated sensor calibration,
for IMU & Odometry, based on laserscan input. 

By launching the calibrate.launch file while the robot is running, 
the calibration node will:
- estimate Imu offset while standing still
- Perform a rotation and estimate linear deviation of IMU and Odometry, based on scan data. 

The robot should be place in front of a straight wall before performing calibration. 
