# joystick_filter

## Brief description

This package filters joystick input, which should be normalized joystick data.
In the settings file it can be chosen whether to use a kalman filter, averaging filter, or both. 
Activating both means the joystick data goes through the averaging filter first, then through the kalman filter. 

Requires joystick input in the form of a Float32MultiArray, with two data points, x & y, normalized and stored as data[0] and data[1]. 
x i.e. data[0] is the in the driving direction of the robot, thus representing linear velocity. 
y i.e. data[1] is perpendicular to the driving direction of the robot, thus representing angular velocity. 

The kalman filter has four states: x, y, vx, vy which are derived from the userinput. 
The prediction model is x' = x + vx*dt

Additionally an exponential function is implemented to increase sensitivity at higher deviations from the middle. Setting the parameter 'control_exponent' to 1 negates the effects (makes it a linear control function).

## Node description

### Parameters
- **debug**: can be used to print debugging info.
- **use_medianfilter**: if "true" activates median (moving average) filter.
- **use_kalmanfilter**: if "true" activates kalman filter.
- **adaptive_kalman**: if "true" continuously updates covariance matrices kalman filter.
- **display_variance_matrices**: if "true" continuously gives a debugging stream of covariance matrix information. 
- **publish_frequency** [Hz]: frequency at which node takes in new user command, filters and publishes the filtered version. 
- **kv**: linear velocity multiplication factor. If the input is normalized (between -1 and 1) and dimensionless, the output is between -kv and +kv **m/s**.
- **kw**: angular velocity multiplication factor. If the input is normalized (between -1 and 1) and dimensionless, the output is between -kw and +kw **rad/s**.
- **x_threshold**: x input below this value is ignored. 
- **y_threshold**: y input below this value is ignored. 
- **control_exponent**: sometimes joystick is made to have an exponentional function, e.g.: v = x^control_exponent. If this is unwanted, set to 1.0
- **buffer_size**: size of medianfilter
- **P0**: Kalman filter uncertainty initialization
- **Hx**: x kalman measurement parameter 
- **Hy**: y kalman measurement parameter
- **cov_x**: x entry to covariance matrix R
- **cov_y**: y entry to covariance matrix R
- **cov_vx**: vx entry to covariance matrix R
- **cov_vy**: vy entry to covariance matrix R
- **Qx**: x entry to covariance matrix Q
- **Qx**: x entry to covariance matrix Q
- **alpha**: learning rate or alternatively forget factor for adaptive Kalman. Higher means longer memory. Between 0 and 1. 

## The following parameters are initial values. They can be adaptively estimated by the Adaptive Kalman Filter by setting adaptive_kalman to true.

### Subscribers

- **/user/joy** (Float32MultiArray):
Normalize joystick input

### Publishers

- **/user/joy/filtered** (Float32MultiArray):
filtered joystick input

- **/user/cmd_vel** (Twist)
filtered joystick input translated into velocity commands

- **/user/performance** (Float32MultiArray):
consists of entries of the [cov_vx, cov_vy, cov_vxy]
