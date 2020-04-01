According to ROS guidelines:
- Also allow negative velocities
- Add a second cost function method which uses the trajectory instead of a single goal position. 
- Don't limit current DWA to v_user. Then if v_user = 0 ,  v = 0, which shouldn't happen. (too sudden braking, user will slide). Instead v_max = max(v_current-v_brake, v_user).
- Member variables should have a trailing  underscore. 
- different acclrt en brake values. 
