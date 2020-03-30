Obstacle_detector package
(Notes by Martijn Krijnen)

Scans merger method:
Uses the transform of each scanner to transform LaserScan message to a pointcloud message.
Adds both pointclouds  together, and transforms back to a single LaserScan.

Obstacle extraction method:
Step 1: group points
- Initialize a new empty group
- Go through the points in Laserscan message.
- If the distance of next point to last point larger than the related parameter setting max_group_distance, it starts a new group and sends previous group to detect_segments()
Step 2: detect segments
- Use linear regression to fit segments 
- Iteratively split segments if certain conditions are met. 
- Iteratively merge if certain conditions are met. 
- Transform segments to circles if smaller than threshold parameter. 
- Iteratively merge circles if circle size smaller than threshold parameter. 

Obstacle tracking method:
- Calculate 2 cost matrices between all new obstacles, and all tracked and untracked obstacles respectively. The cost function represents the difference between each obstacle in terms of location and size. 

Remarks:
The method very explicitly uses the angular order of Laserscan data to detect different obstacles.
Therefore this method is only useful for laserscan data.

The methods used for splitting and mergens different pointgroups, segments and circles are rather heuristic, based on settings, and therefore based on individual laserscan parameters. 
Therefore the package has to be tuned carefully for any new application. 



Potential improvements:
- Use probabilities to indicate confidence in obstacle representation. Let the probability decay over time.
- Approximate the angular velocity of obstacles by taking the change in linear velocity. Use this to improve path approximation. 

