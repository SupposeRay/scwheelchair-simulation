# sparse voronoi

## Brief description
Iteratively calculates the voronoi diagram based on obstacles extracted from the environment. 
Finds the optimal path through the voronoi diagram based on user input. 
There are two ways in which this package can be used:
- As a separate node, just run the sparse_voronoi.launch launch file. 
- By including the VoronoiAlgorithm object in voronoi_algorithm.cpp in another ros node, and passing down information to the object from within the ROS node, using the same interface as in the sparse_voronoi_node.cpp file. This is exhibited in shared_mpc.

Note on terminology: 
The Delaunay triangulation consists of points called 'vertices', and lines connecting them called 'edges'
The Voronoi diagram consists of points called 'nodes', and lines connecting them called 'links'
The Markov Decision Proces consists of 'states', which in this case are the same as the 'nodes' in the Voronoi diagram,
and 'actions', which in this case are the same as the 'links' in the Voronoi diagram. 

## Algorithm
- Perform Constrained Delaunay Triangulation (using CDT or delaunator) library.
- Calculate Incenter of the triangles. These are the voronoi nodes.
- Construct a MDP-graph, by creating a dictionary of node objects. Add connections (defined within node object) between adjacent nodes only if not interrupted by obstacle. 
Connections represent the Actions that can be taken from each Node (=State)
Reward is calculated as the difference between the userinput angle and the angle to the node. (e.g. the more the user points towards a node, the higher the reward).
- Perform value iteration, updating the values of each state.
- Find the optimal path by iteratively selecting the next connected node with the highest reward (greedy policy)
- Interpolate the found path such that it can be used by MPC.

## Node description

### Parameters

- **node_active**: if "false" node remains dormant.
- "debug": if "true" publish debugging info.
- **fixed_frame_id**: 'global' frame_id. 
- **base_frame_id**: 'local' frame_id. 
- **interface_movebase**: if "true", can be used to directly publish the predicted voronoi path to move base as target trajectory. very experimental. NOTE: pub_voropath must also be set to true to enable this feature.
- **pub_frequency** [Hz]: frequency at which algorithm calculates MDP and publishes trajectory.
- **calc_frequency** [Hz]: frequency at which algorithm estimates a new voronoi diagram. 
- **tri_method**: 
    - "constrained": uses Constrained Delaunay Triangulation (recommended, includes constraints of walls in triangulation).
	- "delaunay": fastest triangulation library available (not recommended).
- **mdp_method**: 
	- "qlearning": Optimizes MDP using Q-learning. recommended because it does a second step lookahead and some additional cost for sharp angles is taken into account in this second stap.
	- "value iteration": OPtimizes MDP using Value Iteration.
- **use_dynamic_voronoi**: if "true" activates 'dynamic mode', which calculates multiple voronoi diagrams at different time instants, based on the location prediction of obstacles in the future, and estimates a dynamic path through these 'layers' of voronoi diagrams. 
- **use_dynamic_constraint**: if "true" activates a constraint for connecting two nodes in the dynamic voronoi diagrams, based on whether one node is reachable from the other node in the time-difference (using a max velocity parameter).
- **use_dynamic_reward**: if "true" activates reward calculation based on the last n multiple inputs. Is not always recommended because it introduces a delay. This delay must be carefully managed in combination with e.g. the joystick filter and possibly other components.
- **use_targets**: if "true" targets can be supplied via the topic /sparse_voronoi/targets and these are included as nodes in the voronoi diagram.
- **prediction_horizon**: number of path samples for the interpolated predicted output path.
- **sampling_time**: time between path samples. 
- **max_nr_states**: Maximum number of states/nodes in the predicted voronoi trajectory.
- **max_path_angle**: Maximum change in angle allowed while connecting nodes into the trajectory.
- **pub_markers**: if "true" publishes voronoi and delaunay markers.
- **obst_range** [m]: obstacles outside this range are ignored.
- **min_gap** [m]: minimum gap between obstacles in order to draw a voronoi link.
- **cmd_buffer**: when using 'dynamic reward', determines how many of the last inputs are stored and used for reward calculation.
- **voronoi_horizon**: when using 'dynamic voronoi', determines how many layers of voronoi diagrams (how many points in time) are calculated. (similar to prediction_horizon)
- **voronoi_dt**: when using 'dynamic_voronoi' determines time difference between subsequent voronoi diagrams (similar to sampling_time)
- **max_it**: max number of iterations for solving the MDP
- **min_it**: min number of iterations for solving the MDP
- **it_err_thresh**: error threshold for determining when to stop iterating over the MDP
- **gamma**: gamma parameter in q-learning belman equation
- **alpha**: alpha parameter (learning rate) in value iteration and q-learning
- **cost_angle**: if cost_angle larger than 0.01, adds an extra cost (neg reward) for angles different from current angle wheelchair. Is divided by the distance [m], therefore to have significant influence relative to the other scaled costs (0-1), set cost_angle to ~3 
- **cost_angle2**: if using Qlearning, and cost larger than 0.01, adds an extra cost (neg reward) for abrupt angles in the voronoi diagram. Recommended value: between 0 and 1

### Subscribers

- **/scan_obstacles** (scat_msgs/EnvObjectList):
- **/cam_obstacles** (scat_msgs/EnvObjectList):
Obstacle descriptions. Two subscribers, at each iteration both their inputs are transformed into the local frame and merged. 

- **/user/cmd_vel** (geometry_msgs/Twist):
User input

- **/amcl_pose** [geometry_msgs/PoseWithCovarianceStamped]:
global position

- **/sparse_voronoi/targets** [geometry_msgs/PoseArray]:
Array of potential target poses (e.g. desks, beds, ..)

### Publishers

The following publishers publish visualization messages, used for visualization in rviz. 
- **/sparse_voronoi/voronoi_lines**
- **/sparse_voronoi/delaunay_lines**
- **/sparse_voronoi/voro_path**
- **/sparse_voronoi/user_path**
