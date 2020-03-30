#ifndef SPARSE_VORONOI
#define SPARSE_VORONOI

// ROS
#include <ros/ros.h>
// tf
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
// Messages
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
// C++
#include <stdlib.h>
#include <math.h>
#include <cstdio>
#include <chrono>
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <iso646.h>
#include <deque>
#include <numeric>
// Package
#include <scat_libs/obst_utils.h>
#include <scat_libs/geom_utils.h>
#include <scat_libs/base_utils.h>
#include <scat_libs/rosmsg.h>

#include <scat_libs/diff_drive.h>
#include <scat_msgs/EnvObjectList.h>
#include <scat_msgs/EnvObject.h>

namespace sparse_voronoi
{
using namespace std;

struct State
{
  // Location of Node
  geometry_msgs::Point pos;
  
  // MDP data
  double reward = 0;                    // reward of state (static, = reward of being in a state)
  double value = 0;                     // value of state (dynamic, = rewards possibly gained from this state in future trajectory)
  map<pair<size_t,size_t>, double> Q;   // value of action (= value, dynamically updated during value iteration )
  map<pair<size_t,size_t>, double> R;   // reward of action (static, = reward of following the graph edge from this node to the next)
  pair<size_t,size_t> s_index;          // defined as pair(time,state)
  pair<size_t,size_t> ns_index;         // defined as pair(time,state)
  size_t state;
  size_t time;

  // for trajectory construction
  double edge_steps = 1;  // amount of steps required to reach this node from previous node in trajectory
  double edge_length = 0; // distance between this node and next node
};

template <typename KeyType, typename ValueType>
pair<KeyType, ValueType> get_max(const map<KeyType, ValueType> &x)
{
  using pairtype = pair<KeyType, ValueType>;
  return *max_element(x.begin(), x.end(),
                           [](const pairtype &p1, const pairtype &p2) {
                             return p1.second < p2.second;
                           });
}


class Voronoi_Algorithm
{

public:
  // Constructor
  Voronoi_Algorithm();
  // Destructor
  virtual ~Voronoi_Algorithm();

  // Interface methods (called by Node)
  void initialize();
  void processVoronoi(std_msgs::Header &header);
  void processMDP(geometry_msgs::Pose2D &global_pose2D, geometry_msgs::Twist &target_vel);

  // Methods called from within the processVoronoi and processMDP methods.
  // Can be called in a custom implementation, therefore implemented public methods. 
  void delaunayTriangulation();
  void constrainedTriangulation();
  void buildGraph();
  void buildMarkers();    
  void updateBuffer(double &target_heading, geometry_msgs::Pose2D &global_pose2D);
  void updateRewards();
  void valueIteration();
  void Qlearning();
  void findFullPath();
  void findVoroPath();
  void interpPath();

  //// State containers ////
  std_msgs::Header base_header_;
  // global position
  geometry_msgs::Pose2D agent_pose_;
  // User input twist (local frame)
  geometry_msgs::Twist target_vel_;
  // User input heading (local frame)
  double target_heading_;
  // Obstacle containers. These are used as vertices for the delaunay triangulation. 
  vector<vector<geometry_msgs::Point>> obstacles_pts_;
  scat_msgs::EnvObjectList obstacles_;
  // targets, these are optionally (if use_targets is true) added into the voronoi diagram as nodes (i.e. potential states)
  geometry_msgs::PoseArray voro_targets_;  
  // Path containers
  nav_msgs::Path voro_path_, full_path_;  
  vector<geometry_msgs::Pose2D> interp_path_;
  // For visualization of the Triangulation and Voronoi Diagram
  visualization_msgs::Marker voronoi_lines_, delaunay_lines_;

  //// MDP variables ////
  // list of voronoi nodes (States) at time t, describing a snapshot Voronoi diagram (including node connections)
  map<size_t, State> states_t_;
  // A set of complete lists of voronoi nodes (States), describing a series of snapshot Voronoi diagrams in time.
  map<size_t, map<size_t, State> > states_;
  // list of half-edges at time t: each edge is represented by a pair of pairs: 
  // <pair<size_t,size_t>, pair<size_t,size_t>>
  // key-pair represents a pair of vertices. The vertex indices should be constant throughout the layers of the dynamic voronoi diagram.  
  // value-pair represents a directional node connection, given by he node-object indices connected by this half edge (first -> second). 
  // Similarly to Delaunator, which implements this method using arrays instead of dictionaries.
  map< pair<size_t,size_t>, pair<size_t,size_t> > edges_t_;
  // list of all layers of half-edges 
  map< size_t, map< pair<size_t,size_t>, pair<size_t,size_t> > > edges_;
  // list of vertices at time t
  map< size_t, geometry_msgs::Point > vertices_t_;  
  // list of all layers of vertices
  map< size_t, map<size_t, geometry_msgs::Point> > vertices_; 
  size_t N_states_;


  // lists of previous user input data, for calculation of a reward function with time-history. 
  deque<double> global_userheading_list_;
  deque<double> local_userheading_list_;
  deque<geometry_msgs::Pose2D> global_pose_list_;
  deque<geometry_msgs::Point> local_pos_list_;
  // vertex index (in vertices vector) and triangle/state index (in state dictionary States_)  
  size_t v_agent_, t_agent_; 
  // Container for the trajectory resulting from Value Iteration (in valueIteration()), containing States
  vector<pair<size_t,size_t>> voropath_states_;
  vector<pair<size_t,size_t>> fullpath_states_;

  //// Parameters ////
  // Typical booleans deciding whether to publish these topics.
  bool pub_markers_ = true;
  bool voronoi_available_ = false;
  bool use_dynamic_voronoi_ = false;
  bool use_dynamic_constraint_ = false;
  bool use_dynamic_reward_ = false;
  bool use_targets_ = false;
  // Some method choice parameters
  // Triangulation method. "delaunay" (fast) or "constrained" (takes line semgent constraints into account)
  string tri_method_ = "constrained";
  // mdp method: valueiteration or qlearning
  string mdp_method_ = "valueiteration";
  // prediction horizon, used for trajectory generation 
  double prediction_horizon_ = 6;
  // time interval, used for trajectory generation 
  double time_interval_ = 0.4;
  // voronoi prediction horizon: determines nr. of voronoi diagrams to be made
  int voronoi_horizon_ = 5;
    // time interval for dynamic voronoi
  double voronoi_dt_ = 0.3;
  // frame id's
  string fixed_frame_id_ = "map", base_frame_id_ = "base_footprint";
  // The minimum gap which the robot needs to pass through.
  // If node connections pass through a gap smaller than this, the voronoi connection should not be drawn.
  double min_gap_ = 0.7;
  // gamma is the learning rate for the value iteration learning algorithm.
  double gamma_ = 0.8;
  // alpha is the learning rate of Q-learning algorithm.
  double alpha_ = 0.8;
  // Maximum number of iterations for value iterations=
  int max_it_ = 10;
  int min_it_ = 3;
  // Number of user input commands which are considered in reward calculation
  int cmd_buffersize_ = 5;
  // weights for user input. Recent ones get a higher weight than older ones.
  vector<double> cmd_weights_;
  // termination error threshold value iteration
  double it_err_thresh_ = 0.01;
  // obstacle frange, obstacles farther away than this are not considered.
  double obst_range_ = 10.0;
  // Cost factor used as multiplier for the cost due to misalignment with the current robot direction
  double cost_angle_ = 0.5;
  // Cost factor used as multiplier for the cost due to angle between two subsequent segments (only for Q-learning)
  double cost_angle2_ = 0.5;
  // max nr of states in trajectory
  int max_nr_states_ = 6;
  // max angle allowed in trajectory
  double max_path_angle_ = M_PI/2;
  
private:
  // Private Methods
  pair<size_t, State> createNode(size_t time, size_t t, geometry_msgs::Point point);
  void addAction(size_t& time1, size_t& tri1, size_t& time2, size_t& tri2);
  double calcReward(geometry_msgs::Point node);
  bool noIntersections(geometry_msgs::Point t1, geometry_msgs::Point t2);

  // tf listener
  tf::TransformListener tf_listener_;
};

} // namespace sparse_voronoi
#endif