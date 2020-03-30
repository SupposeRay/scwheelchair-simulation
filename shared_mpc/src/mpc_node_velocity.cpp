#include "shared_mpc/mpc_node_velocity.h"

namespace shared_mpc
{

MPCNode::MPCNode(ros::NodeHandle &node_handle)
    : node_handle_(node_handle)
{
  ROS_DEBUG("Launching Shared MPC");

  if (!readParameters() || !readBotParameters() || !readIpoptParameters())
  {
      ROS_ERROR("Could not load parameters.");
      ros::requestShutdown();
  }

  initialize();

  // Subscribers & Publishers
  userinput_subscriber_ = node_handle_.subscribe("/user/cmd_vel", 1, &MPCNode::userinputCallback, this);
  odom_subscriber_ = node_handle_.subscribe("/odom", 1, &MPCNode::odomCallback, this);
  laser_obstacle_subscriber_ = node_handle_.subscribe("/scan_obstacles", 1, &MPCNode::laserObjectCallback, this);
  cam_obstacle_subscriber_ = node_handle_.subscribe("/test_obst_msg", 1, &MPCNode::camObjectCallback, this);

  if (pos_method_ == "global")
    position_subscriber_ = node_handle_.subscribe("/amcl_pose", 1, &MPCNode::positionCallback, this);

  if (use_adaptive_)
    userperf_subscriber_ = node_handle_.subscribe("/user/performance", 1, &MPCNode::perfCallback, this);

  if (use_voronoi_)
  {
    if (!readVoronoiParameters())
    {
      ROS_ERROR("Could not load Voronoi parameters.");
      ros::requestShutdown();
    }
    voronoi_algorithm_.initialize();
    if (voronoi_algorithm_.pub_markers_)
    {
      vorolines_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("voronoi_lines", 1);
      delaunaylines_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("delaunay_lines", 1);
    }
    if (use_voronoi_)
      voropath_publisher_ = node_handle_.advertise<nav_msgs::Path>("voro_path", 1);
  }

  cmd_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  user_path_publisher_ = node_handle_.advertise<nav_msgs::Path>("user_path", 1);
  mpc_path_publisher_ = node_handle_.advertise<nav_msgs::Path>("mpc_path", 1);

  activation_service_ = node_handle_.advertiseService("activation_service", &MPCNode::serviceCallback, this);

  if (node_active_)
  {
    while (!(laser_obstacles_received_ || cam_obstacles_received_))
    {
      ROS_INFO("shared mpc waiting for obstacle data");
      ros::Duration(1).sleep();
      ros::spinOnce();
    }
    if (pos_method_ == "global")
    {
      while (!pos_received_)
      {
        ROS_INFO("shared mpc waiting for position data");
        ros::Duration(1).sleep();
        ros::spinOnce();
      }
    }
    while (!odom_received_)
    {
      ROS_INFO("shared mpc waiting for odometry data");
      ros::Duration(1).sleep();
      ros::spinOnce();
    }
  }

  base_header_.stamp = ros::Time::now();
  base_header_.frame_id = base_frame_id_;

  ROS_INFO("Shared mpc all info received");
}

MPCNode::~MPCNode() {}

bool MPCNode::readBotParameters()
{
  if (!node_handle_.getParam("v_min", v_min_))
    ROS_WARN_STREAM("Parameter v_min not set for shared_mpc. Using default setting: " << v_min_);
  if (!node_handle_.getParam("v_max", v_max_))
    ROS_WARN_STREAM("Parameter v_max not set for shared_mpc. Using default setting: " << v_max_);
  if (!node_handle_.getParam("v_max_neg", v_max_neg_))
    ROS_WARN_STREAM("Parameter v_max_neg not set for shared_mpc. Using default setting: " << v_max_neg_);
  if (!node_handle_.getParam("v_threshold", v_threshold_))
    ROS_WARN_STREAM("Parameter v_threshold not set for shared_mpc. Using default setting: " << v_threshold_);
  if (!node_handle_.getParam("w_max", w_max_))
    ROS_WARN_STREAM("Parameter w_max not set for shared_mpc. Using default setting: " << w_max_);
  if (!node_handle_.getParam("w_threshold", w_threshold_))
    ROS_WARN_STREAM("Parameter w_threshold not set for shared_mpc. Using default setting: " << w_threshold_);
  if (!node_handle_.getParam("acc_lim_x", v_acc_))
    ROS_WARN_STREAM("Parameter acc_lim_x not set for shared_mpc. Using default setting: " << v_acc_);
  if (!node_handle_.getParam("dec_lim_x", v_dec_))
    ROS_WARN_STREAM("Parameter dec_lim_x not set for shared_mpc. Using default setting: " << v_dec_);
  if (!node_handle_.getParam("acc_lim_theta", w_acc_))
    ROS_WARN_STREAM("Parameter acc_lim_theta not set for shared_mpc. Using default setting: " << w_acc_);
  if (!node_handle_.getParam("dec_lim_theta", w_dec_))
    ROS_WARN_STREAM("Parameter dec_lim_theta not set for shared_mpc. Using default setting: " << w_dec_);
  if (!node_handle_.getParam("base_width", base_width_))
    ROS_WARN_STREAM("Parameter base_width not set for shared_mpc. Using default setting: " << base_width_);
  if (!node_handle_.getParam("robot_radius", robot_radius_))
    ROS_WARN_STREAM("Parameter robot_radius not set for shared_mpc. Using default setting: " << robot_radius_);
      
  if (v_max_neg_ > 0)
  {
    ROS_ERROR_STREAM("v_max_neg is larger than zero. This should be a negative number.");
    return false;
  }

  w_min_ = 2 * v_min_ / base_width_;

  return true;
}

bool MPCNode::readIpoptParameters()
{
  double max_cpu_time, tol, dual_inf_tol, compl_inf_tol, constr_viol_tol, bound_frac, bound_push, slack_bound_frac, slack_bound_push, bound_relax_factor;
  int print_level, max_iter;
  std::string mu_strategy, hessian_approximation, linear_solver;

  //// Options for IPOPT solver ////
  solver_options_.clear();
  solver_options_ += "Sparse  true        forward\n";
  solver_options_ += "String expect_infeasible_problem  yes\n";
  // solver_options_ += "String derivative_test second-order\n";
  if (node_handle_.getParam("print_level", print_level))
    solver_options_ += "Integer print_level " + std::to_string(print_level) + " \n";
  if (node_handle_.getParam("max_cpu_time", max_cpu_time))
    solver_options_ += "Numeric max_cpu_time " + std::to_string(max_cpu_time) + " \n";
  if (node_handle_.getParam("max_iter", max_iter))
    solver_options_ += "Integer max_iter " + std::to_string(max_iter) + " \n";
  if (node_handle_.getParam("tol", tol))
    solver_options_ += "Numeric tol " + std::to_string(tol) + " \n";
  if (node_handle_.getParam("dual_inf_tol", dual_inf_tol))
    solver_options_ += "Numeric dual_inf_tol " + std::to_string(dual_inf_tol) + " \n";
  if (node_handle_.getParam("compl_inf_tol", compl_inf_tol))
    solver_options_ += "Numeric compl_inf_tol " + std::to_string(compl_inf_tol) + " \n"; // Desired complementarity conditions violation, condition for succesful termination
  if (node_handle_.getParam("constr_viol_tol", constr_viol_tol))
    solver_options_ += "Numeric constr_viol_tol " + std::to_string(constr_viol_tol) + " \n"; // Desired contraint violation, condition for succesful termination
  if (node_handle_.getParam("bound_frac", bound_frac))
    solver_options_ += "Numeric bound_frac " + std::to_string(bound_frac) + " \n";
  if (node_handle_.getParam("bound_push", bound_push))
    solver_options_ += "Numeric bound_push " + std::to_string(bound_push) + " \n";
  if (node_handle_.getParam("slack_bound_frac", slack_bound_frac))
    solver_options_ += "Numeric slack_bound_frac " + std::to_string(slack_bound_frac) + " \n";
  if (node_handle_.getParam("slack_bound_push", slack_bound_push))
    solver_options_ += "Numeric slack_bound_push " + std::to_string(slack_bound_push) + " \n";
  if (node_handle_.getParam("bound_relax_factor", bound_relax_factor))
    solver_options_ += "Numeric bound_relax_factor " + std::to_string(bound_relax_factor) + " \n";
  // if (node_handle_.getParam("mu_strategy", mu_strategy))
  //   solver_options_ += "String mu_strategy " + mu_strategy + " \n";
  // if (node_handle_.getParam("hessian_approximation", hessian_approximation))
  //   solver_options_ += "String hessian_approximation " + hessian_approximation + " \n";
  // if (node_handle_.getParam("linear_solver", linear_solver))
  // solver_options_ += "String linear_solver " + linear_solver + "\n";
  // solver_options_ += "String linear_solver ma57\n";

  return true;
}

bool MPCNode::readParameters()
{
  //// MPC parameters ////
  if (!node_handle_.getParam("node_active", node_active_))
    ROS_WARN_STREAM("Parameter node_active not set for shared_mpc. Using default setting: " << node_active_);
  if (!node_handle_.getParam("base_frame_id", base_frame_id_))
    ROS_WARN_STREAM("Parameter base_frame_id not set for shared_mpc. Using default setting: " << base_frame_id_);
  if (!node_handle_.getParam("fixed_frame_id", fixed_frame_id_))
    ROS_WARN_STREAM("Parameter fixed_frame_id not set for shared_mpc. Using default setting: " << fixed_frame_id_);
  if (!node_handle_.getParam("use_voronoi", use_voronoi_))
    ROS_WARN_STREAM("Parameter use_voronoi not set for shared_mpc. Using default setting: " << use_voronoi_);
  if (!node_handle_.getParam("use_corner_constraints", use_corner_constraints_))
    ROS_WARN_STREAM("Parameter use_corner_constraints not set for shared_mpc. Using default setting: " << use_corner_constraints_);
  if (!node_handle_.getParam("use_adaptive", use_adaptive_))
    ROS_WARN_STREAM("Parameter use_adaptive not set for shared_mpc. Using default setting: " << use_adaptive_);
  if (!node_handle_.getParam("pos_method", pos_method_))
    ROS_WARN_STREAM("Parameter pos_method not set for shared_mpc. Using default setting: " << pos_method_);
  if (!node_handle_.getParam("prediction_horizon", prediction_horizon_))
    ROS_WARN_STREAM("Parameter prediction_horizon not set for shared_mpc. Using default setting: " << prediction_horizon_);
  if (!node_handle_.getParam("sampling_time", time_interval_))
    ROS_WARN_STREAM("Parameter sampling_time not set for shared_mpc. Using default setting: " << time_interval_);
  if (!node_handle_.getParam("obst_dist_threshold", obst_dist_threshold_))
    ROS_WARN_STREAM("Parameter obst_dist_threshold not set for shared_mpc. Using default setting: " << obst_dist_threshold_);
  if (!node_handle_.getParam("obst_range", obst_range_))
    ROS_WARN_STREAM("Parameter obst_range not set for shared_mpc. Using default setting: " << obst_range_);
  if (!node_handle_.getParam("obst_frequency", obst_frequency_))
    ROS_WARN_STREAM("Parameter obst_frequency not set for shared_mpc. Using default setting: " << obst_frequency_);
  if (!node_handle_.getParam("calc_frequency", calc_frequency_))
    ROS_WARN_STREAM("Parameter calc_frequency not set for shared_mpc. Using default setting: " << calc_frequency_);

  // Load constants
  if (use_adaptive_)
  {
    ROS_INFO_STREAM("Shared MPC is using Adaptive mode, Cost function tuned to individual users based on joystick filter parameters.");
    if (!node_handle_.getParam("Kv0", Kv0_))
      ROS_WARN_STREAM("Parameter Kv0 not set for shared_mpc. Using default setting: " << Kv0_);
    if (!node_handle_.getParam("Kw0", Kw0_))
      ROS_WARN_STREAM("Parameter Kw0 not set for shared_mpc. Using default setting: " << Kw0_);
    if (!node_handle_.getParam("Kh0", Kh0_))
      ROS_WARN_STREAM("Parameter Kh0 not set for shared_mpc. Using default setting: " << Kh0_);
    if (!node_handle_.getParam("Kvr", Kvr_))
      ROS_WARN_STREAM("Parameter Kvr not set for shared_mpc. Using default setting: " << Kvr_);
    if (!node_handle_.getParam("Kwr", Kwr_))
      ROS_WARN_STREAM("Parameter Kwr not set for shared_mpc. Using default setting: " << Kwr_);
    if (!node_handle_.getParam("Khr", Khr_))
      ROS_WARN_STREAM("Parameter Khr not set for shared_mpc. Using default setting: " << Khr_);
    Kv_ = Kv0_ + Kvr_;
    Kw_ = Kw0_ + Kwr_;
    Kh_ = Kh0_ + Khr_;
  }
  else
  {
    if (!node_handle_.getParam("Kv", Kv_))
      ROS_WARN_STREAM("Parameter Kv not set for shared_mpc. Using default setting: " << Kv_);
    if (!node_handle_.getParam("Kw", Kw_))
      ROS_WARN_STREAM("Parameter Kw not set for shared_mpc. Using default setting: " << Kw_);
    if (!node_handle_.getParam("Kh", Kh_))
      ROS_WARN_STREAM("Parameter Kh not set for shared_mpc. Using default setting: " << Kh_);
  }
  if (!node_handle_.getParam("Ks", Ks_))
    ROS_WARN_STREAM("Parameter Ks not set for shared_mpc. Using default setting: " << Ks_);
  if (!node_handle_.getParam("Keq", Keq_))
    ROS_WARN_STREAM("Parameter Keq not set for shared_mpc. Using default setting: " << Keq_);
  if (!node_handle_.getParam("Kineq", Kineq_))
    ROS_WARN_STREAM("Parameter Kineq not set for shared_mpc. Using default setting: " << Kineq_);

  bool debug = false;
  if (!node_handle_.getParam("debug", debug))
    ROS_WARN_STREAM("Parameter debug not set for sparse_voronoi. Using default setting: " << debug);  
  // When debugging, set logger_level to Debug by changing Info to Debug.
  if (debug)
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  else
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
      
  ROS_DEBUG("Parameters read.");
  return true;
}

bool MPCNode::readVoronoiParameters()
{
  // Parameters for Voronoi Algorithm
  if (!node_handle_.getParam("pub_markers", voronoi_algorithm_.pub_markers_))
    ROS_WARN_STREAM("Parameter pub_markers not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.pub_markers_);
  if (!node_handle_.getParam("tri_method", voronoi_algorithm_.tri_method_))
    ROS_WARN_STREAM("Parameter tri_method not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.tri_method_);
  if (!node_handle_.getParam("mdp_method", voronoi_algorithm_.mdp_method_))
    ROS_WARN_STREAM("Parameter mdp_method not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.mdp_method_);
  if (!node_handle_.getParam("cmd_buffer", voronoi_algorithm_.cmd_buffersize_))
    ROS_WARN_STREAM("Parameter cmd_buffer not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.cmd_buffersize_);
  if (!node_handle_.getParam("max_it", voronoi_algorithm_.max_it_))
    ROS_WARN_STREAM("Parameter max_it not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.max_it_);
  if (!node_handle_.getParam("gamma", voronoi_algorithm_.gamma_))
    ROS_WARN_STREAM("Parameter gamma not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.gamma_);
  if (!node_handle_.getParam("alpha", voronoi_algorithm_.alpha_))
    ROS_WARN_STREAM("Parameter alpha not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.alpha_);
  if (!node_handle_.getParam("min_it", voronoi_algorithm_.min_it_))
    ROS_WARN_STREAM("Parameter min_it not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.min_it_);
  if (!node_handle_.getParam("it_err_thresh", voronoi_algorithm_.it_err_thresh_))
    ROS_WARN_STREAM("Parameter it_err_thresh not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.it_err_thresh_);
  if (!node_handle_.getParam("min_gap", voronoi_algorithm_.min_gap_))
    ROS_WARN_STREAM("Parameter min_gap not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.min_gap_);
  if (!node_handle_.getParam("cost_angle", voronoi_algorithm_.cost_angle_))
    ROS_WARN_STREAM("Parameter cost_angle not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.cost_angle_);
  if (!node_handle_.getParam("cost_angle2", voronoi_algorithm_.cost_angle2_))
    ROS_WARN_STREAM("Parameter cost_angle2 not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.cost_angle2_);
  if (!node_handle_.getParam("use_dynamic_voronoi", voronoi_algorithm_.use_dynamic_voronoi_))
    ROS_WARN_STREAM("Parameter use_dynamic_voronoi not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.use_dynamic_voronoi_);
  if (!node_handle_.getParam("use_dynamic_constraint", voronoi_algorithm_.use_dynamic_constraint_))
    ROS_WARN_STREAM("Parameter use_dynamic_constraint not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.use_dynamic_constraint_);
  if (!node_handle_.getParam("use_dynamic_constraint", voronoi_algorithm_.use_dynamic_constraint_))
    ROS_WARN_STREAM("Parameter use_dynamic_constraint not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.use_dynamic_constraint_);
  if (!node_handle_.getParam("max_path_angle", voronoi_algorithm_.max_path_angle_))
    ROS_WARN_STREAM("Parameter max_path_angle not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.max_path_angle_);
  if (!node_handle_.getParam("max_nr_states", voronoi_algorithm_.max_nr_states_))
    ROS_WARN_STREAM("Parameter max_nr_states not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.max_nr_states_);
  if (!node_handle_.getParam("use_dynamic_reward", voronoi_algorithm_.use_dynamic_reward_))
    ROS_WARN_STREAM("Parameter use_dynamic_reward not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.use_dynamic_reward_);

  if (voronoi_algorithm_.use_dynamic_voronoi_)
  {
    if (!node_handle_.getParam("voronoi_horizon", voronoi_algorithm_.voronoi_horizon_))
      ROS_WARN_STREAM("Parameter voronoi_horizon not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.voronoi_horizon_);
    if (!node_handle_.getParam("voronoi_dt", voronoi_algorithm_.voronoi_dt_))
      ROS_WARN_STREAM("Parameter voronoi_dt not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.voronoi_dt_);
  }
  voronoi_algorithm_.obst_range_ = obst_range_;
  voronoi_algorithm_.fixed_frame_id_ = fixed_frame_id_;
  voronoi_algorithm_.base_frame_id_ = base_frame_id_;
  voronoi_algorithm_.prediction_horizon_ = prediction_horizon_;
  voronoi_algorithm_.time_interval_ = time_interval_;

  return true;
}

void MPCNode::userinputCallback(const geometry_msgs::Twist::ConstPtr &msg_user)
{
  if (node_active_)
  {
    cmd_received_ = true;
    v_user_ = std::max(std::min(v_max_, msg_user->linear.x), v_max_neg_);
    w_user_ = std::max(std::min(w_max_, msg_user->angular.z), -w_max_);
    t_cmd_now_ = ros::Time::now();
    ros::Duration dt = t_cmd_now_ - t_cmd_prev_;
    if (dt.toSec() > 1.0 / calc_frequency_)
    {
      t_cmd_prev_ = t_cmd_now_;
      if ((laser_obstacles_received_ || cam_obstacles_received_) && odom_received_ && (fabs(v_user_) > v_threshold_ || fabs(w_user_) > w_threshold_))
      {
        // current control inputs
        std::vector<double> U0{v_agent_, w_agent_};
        // current user (reference) control inputs
        std::vector<double> U_ref{v_user_, w_user_};
        // process obstacles: Translates the latest received obstacle messages stored as scat_msgs::EnvObstacleList format into:
        // vectors of 2D points in local frame, for Voronoi diagram calculation
        // AD vectors of AD parameters (AD = Algorithm Differentiation, format required for IPOPT MPC)
        processObstacles();
        // collect obstacles which are used for MPC in a single structure and poss to MPC solver
        std::vector<CppAD::vector<CppAD::vector<AD<double>>>> obstacles = {circles_, ellipses_};
        // call MPC solver
        solveMPC(U0, U_ref, obstacles);
      }
      else
      {
        cmd_publisher_.publish(rosmsg::makeTwist(0.0, 0.0));
      }
    }
  }
}

void MPCNode::perfCallback(const std_msgs::Float32MultiArray::ConstPtr &msg_perf)
{
  if (node_active_ && use_adaptive_)
  {
    float var_x = std::max(std::min(msg_perf->data[0], 1.0f), 0.0f);
    float var_y = std::max(std::min(msg_perf->data[1], 1.0f), 0.0f);
    float cov_xy = std::max(std::min(msg_perf->data[2], 1.0f), 0.0f);
    fg_eval_.Kv_ = Kv0_ + Kvr_ * (1 - var_x);
    fg_eval_.Kw_ = Kw0_ + Kwr_ * (1 - var_y);
    fg_eval_.Kh_ = Kh0_ + Khr_ * (1 - cov_xy);
  }
}

void MPCNode::positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr  &msg_position)
{
  if (node_active_)
  {
    global_pose2D_.x = msg_position->pose.pose.position.x;
    global_pose2D_.y = msg_position->pose.pose.position.y;
    global_pose2D_.theta = tf::getYaw(msg_position->pose.pose.orientation);
    pos_received_ = true;
  }
}

void MPCNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom)
{
  if (node_active_)
  {
    v_agent_ = msg_odom->twist.twist.linear.x;
    w_agent_ = msg_odom->twist.twist.angular.z;
    if (pos_method_ == "odom")
    {
      global_pose2D_.x = msg_odom->pose.pose.position.x;
      global_pose2D_.y = msg_odom->pose.pose.position.y;
      global_pose2D_.theta = tf::getYaw(msg_odom->pose.pose.orientation);
    }
    odom_received_ = true;
  }
}

void MPCNode::laserObjectCallback(const scat_msgs::EnvObjectList::ConstPtr &msg_objects)
{
  if (node_active_)
  {
    laser_object_msg_ = obst_utils::transformObjectList(*msg_objects, base_frame_id_, tf_listener_);
    laser_obstacles_received_ = true;
    // ROS_INFO_STREAM("laser obstacles received: " << msg_objects.objects.size());
  }
}

void MPCNode::camObjectCallback(const scat_msgs::EnvObjectList::ConstPtr &msg_objects)
{
  if (node_active_)
  {
    cam_object_msg_ = obst_utils::transformObjectList(*msg_objects, base_frame_id_, tf_listener_);
    cam_obstacles_received_ = true;
    // ROS_INFO_STREAM("laser obstacles received: " << msg_objects.objects.size());
  }
}

void MPCNode::initialize()
{
  t_cmd_prev_ = ros::Time::now();
  t_obst_prev_ = ros::Time::now();
  //// Set Parameters ////
  Np_ = prediction_horizon_;
  Np_d_ = static_cast<double>(Np_);
  constants_[0] = Kv_ / Np_d_; // linear velocity
  constants_[1] = Kw_ / Np_d_; // angular velocity
  constants_[2] = Kh_ / Np_d_; // heading
  constants_[3] = Ks_ / Np_d_; // smoothness
  constants_[4] = Keq_;        // equality constraints of model
  constants_[5] = Kineq_;      // inequality constraints of obstacles

  // Set the number of model variables (includes both states and inputs).
  n_vars_ = Np_ * n_states_ + (Np_ - 1) * n_inputs_;
  // Set indices for variable containers.
  // Used in variables and constraints
  ind_x_ = 0 * Np_;
  ind_y_ = 1 * Np_;
  ind_theta_ = 2 * Np_;
  // Used only in variables, and not for the last state
  ind_v_ = 3 * Np_;
  ind_w_ = 4 * Np_ - 1;
  // Settings for cost & constraints evaluation object
  std::vector<size_t> indices{ind_x_, ind_y_, ind_theta_, ind_v_, ind_w_};
  fg_eval_.setParams(indices, constants_, v_max_, w_max_, Np_, time_interval_, n_vars_);

  double max_distance = (Np_d_ + 1) * time_interval_ * v_max_;
  double max_rot = (Np_d_ + 1) * time_interval_ * w_max_;
  //// Set bounds on Variables ////
  vars_ = Dvector(n_vars_);
  vars_lowerbound_ = Dvector(n_vars_);
  vars_upperbound_ = Dvector(n_vars_);
  size_t i;

  for (i = ind_x_; i < ind_y_; i++)
  {
    vars_lowerbound_[i] = -1e2;
    vars_upperbound_[i] = 1e2;
  }
  // Set lower and upper limits for Y.
  for (i = ind_y_; i < ind_theta_; i++)
  {
    vars_lowerbound_[i] = -1e2;
    vars_upperbound_[i] = 1e2;
  }
  // Set lower and upper limits for Theta.
  for (i = ind_theta_; i < ind_v_; i++)
  {
    vars_lowerbound_[i] = -M_PI;
    vars_upperbound_[i] = M_PI;
  }
  vars_lowerbound_[ind_x_] = 0.0;
  vars_upperbound_[ind_x_] = 0.0;
  vars_lowerbound_[ind_y_] = 0.0;
  vars_upperbound_[ind_y_] = 0.0;
  vars_lowerbound_[ind_theta_] = 0.0;
  vars_upperbound_[ind_theta_] = 0.0;

  // Alternative bounds: Dynamic window type constraints
  // for (size_t t = 1; t < Np_; t++)
  // {
  //   vars_lowerbound_[t+ind_x_] = vars_lowerbound_[t-1+ind_x_] - v_max_ * time_interval_;
  //   vars_upperbound_[t+ind_x_] = vars_upperbound_[t-1+ind_x_] + v_max_ * time_interval_;
  //   vars_lowerbound_[t+ind_y_] = vars_lowerbound_[t-1+ind_y_] - v_max_ * time_interval_;
  //   vars_upperbound_[t+ind_y_] = vars_upperbound_[t-1+ind_y_] + v_max_ * time_interval_;
  //   vars_lowerbound_[t+ind_theta_] = -M_PI;
  //   vars_upperbound_[t+ind_theta_] = M_PI;
  // }
  // for (size_t t = 1; t < Np_ - 2; t++)
  // {
  //   vars_lowerbound_[t+ind_x_] = -1e3;
  //   vars_upperbound_[t+ind_x_] = 1e3;
  //   vars_lowerbound_[t+ind_y_] = -1e3;
  //   vars_upperbound_[t+ind_y_] = 1e3;
  //   vars_lowerbound_[t+ind_theta_] = -M_PI;
  //   vars_upperbound_[t+ind_theta_] = M_PI;
  // }

  // // The upper and lower limits for translational velocity
  // for (i = ind_v_; i < ind_w_; i++)
  // {
  //   vars_lowerbound_[i] = v_max_neg_;
  //   vars_upperbound_[i] = v_max_;
  // }

  // // The upper and lower limits for rotational velocity
  // for (i = ind_w_; i < n_vars_; i++)
  // {
  //   vars_lowerbound_[i] = -w_max_;
  //   vars_upperbound_[i] = w_max_;
  // }

  ROS_INFO("Parameter initialization finished.");
}

void MPCNode::solveMPC(std::vector<double> &U0,
                       std::vector<double> &U_ref,
                       std::vector<CppAD::vector<CppAD::vector<AD<double>>>> &obstacles)
{
  ROS_DEBUG_STREAM("Starting MPC solver");
  ROS_DEBUG_STREAM("Kv: " << fg_eval_.Kv_ << "Kh: " << fg_eval_.Kh_);
  ROS_DEBUG_STREAM("U0v: " << U0[0] << " U0w: " << U0[1]);
  ROS_DEBUG_STREAM("U_ref: " << U_ref[0] << " U_ref: " << U_ref[1]);

  base_header_.stamp = ros::Time::now();
  start_time_ = std::chrono::system_clock::now();
  std::vector<double> X0(3, 0.0);
  //// Construct user path
  user_path_ = diff_drive::makePath(U_ref[0], U_ref[1], Np_, time_interval_, base_header_);

  if (use_voronoi_)
  {
    ROS_WARN_STREAM("Using voronoi path as initial guess for MPC");
    // Perform triangulation
    geometry_msgs::Twist cmd_user = rosmsg::makeTwist(v_user_,w_user_);
    voronoi_algorithm_.processVoronoi(base_header_);
    voronoi_algorithm_.processMDP(global_pose2D_, cmd_user);
    // Load the optimal path, according to the voronoi diagram and cmd_vel
    // loadPath(voronoi_algorithm_.interp_path_, 0, 0);
    loadPath(voronoi_algorithm_.interp_path_, U_ref[0], U_ref[1]);
    // loadPath(voronoi_algorithm_.interp_path_, U0[0], U0[1]);
  }
  else
  {
    // Not using Voronoi method. If previous iteration succeeded, load path from previous estimate:
    if (succes_ && fg_eval_.constr_viols_ == false)
    {
      ROS_DEBUG_STREAM("Previous iteration succesful. Using warm start method");
      loadWarmStartPath(U_ref[0], U_ref[1]);
      // loadWarmStartPath(U0[0], U0[1]);
    }
    else // Start from zero
    {
      ROS_DEBUG_STREAM("Previous iteration failed! Starting iteration from 0");
      loadPath(diff_drive::makePath(0, 0, Np_, time_interval_, base_header_), 0, 0);
      // loadPath(diff_drive::makePath(U_ref[0], U_ref[1], Np_, time_interval_, base_header_), U_ref[0], U_ref[1]);
      // loadPath(diff_drive::makePath(U0[0], U0[1], Np_, time_interval_, base_header_), U0[0], U0[1]);
    }
  }

  // ROS_DEBUG_STREAM("Set initial guess IPOPT");
  //// Set initial guess MPC algorithm

  // ROS_DEBUG_STREAM("Set bounds");
  //// Set bounds on Controls ////
  // Use dynamic window to limit the velocities throughout the prediction horizon
  // The upper and lower limits for translational velocity
  // ROS_WARN_STREAM("w_ref: " << U_ref[1]);
  // ROS_WARN_STREAM("w: " << U0[1]);
  double v_min, v_max;
  if (U_ref[0] > v_threshold_)
  {
    v_min = 0;
    v_max = v_max_;
  }
  else if (U_ref[0] < -v_threshold_)
  {
    v_min = v_max_neg_;
    v_max = 0;
  }
  else
  {
    v_min = 0;
    v_max = 0;
  }
  // Set Input bounds for t = 0
  // vars_lowerbound_[ind_v_] = U0[0];
  // vars_upperbound_[ind_v_] = U0[0];
  // vars_lowerbound_[ind_w_] = U0[1];
  // vars_upperbound_[ind_w_] = U0[1];
  vars_lowerbound_[ind_v_] = std::max(std::min(U0[0], v_max) - v_dec_ * time_interval_, v_min);
  vars_upperbound_[ind_v_] = std::min(std::max(U0[0], v_min) + v_acc_ * time_interval_, v_max);
  vars_lowerbound_[ind_w_] = std::max(std::min(U0[1], w_max_) - w_dec_ * time_interval_, -w_max_);
  vars_upperbound_[ind_w_] = std::min(std::max(U0[1], -w_max_) + w_acc_ * time_interval_, w_max_);

  // Set Input bounds for trajectory based on acceleration/deceleration & velocity limits.
  for (size_t t = 1; t < Np_ - 1; t++)
  {
    vars_lowerbound_[t + ind_v_] = std::max(std::min(vars_lowerbound_[t - 1 + ind_v_], v_max) - v_dec_ * time_interval_, v_min);
    vars_upperbound_[t + ind_v_] = std::min(std::max(vars_upperbound_[t - 1 + ind_v_], v_min) + v_acc_ * time_interval_, v_max);
    vars_lowerbound_[t + ind_w_] = std::max(std::min(vars_lowerbound_[t - 1 + ind_w_], w_max_) - w_dec_ * time_interval_, -w_max_);
    vars_upperbound_[t + ind_w_] = std::min(std::max(vars_upperbound_[t - 1 + ind_w_], -w_max_) + w_acc_ * time_interval_, w_max_);
  }

  //// Constraints ////
  CppAD::vector<CppAD::vector<AD<double>>> circles = obstacles[0];
  CppAD::vector<CppAD::vector<AD<double>>> ellipses = obstacles[1];

  // Set the number of constraints
  n_constraints_ = Np_ * n_states_ + Np_ * (circles.size() + ellipses.size());
  // Constraint indices
  ind_circles_ = ind_theta_ + Np_;
  ind_ellipses_ = ind_circles_ + Np_ * circles.size();
  //// Set bounds on Constraints ////
  // Lower and upper limits for the constraints
  Dvector constraints_lowerbound_ = Dvector(n_constraints_);
  Dvector constraints_upperbound_ = Dvector(n_constraints_);
  //// Initialize constraints:
  // model constraints are equality constraints: h = 0
  // therefore both lb and ub are set to 0 (except initial state which is defined in solveMPC())
  for (size_t i = 0; i < ind_circles_; i++)
  {
    constraints_lowerbound_[i] = 0;
    constraints_upperbound_[i] = 0;
  }
  /// obstacle constraints are inequality constraints: g < 0
  for (size_t i = ind_circles_; i < n_constraints_; i++)
  {
    constraints_lowerbound_[i] = -1e10;
    constraints_upperbound_[i] = 0;
  }

  // for (int i = 0; i < vars_lowerbound_.size(); i++)
  // {
  //   ROS_WARN_STREAM(" LB:  " << vars_lowerbound_[i] << "  var0:  " << vars_[i] << "  UB:  " << vars_upperbound_[i]);
  // }

  // Set the initial state constraint values
  constraints_lowerbound_[ind_x_] = X0[0];
  constraints_lowerbound_[ind_y_] = X0[1];
  constraints_lowerbound_[ind_theta_] = X0[2];

  constraints_upperbound_[ind_x_] = X0[0];
  constraints_upperbound_[ind_y_] = X0[1];
  constraints_upperbound_[ind_theta_] = X0[2];

  // update the object that computes objective and constraints
  fg_eval_.update(U0, U_ref, n_constraints_, ind_circles_, ind_ellipses_, circles, ellipses);


  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      solver_options_, vars_, vars_lowerbound_, vars_upperbound_, constraints_lowerbound_,
      constraints_upperbound_, fg_eval_, solution_);

  // ROS_DEBUG_STREAM("finished solver");
  // ROS_DEBUG_STREAM("constraints size: " << n_constraints_);
  // ROS_DEBUG_STREAM("constraints_lowerbound_ size: " << constraints_lowerbound_.size());
  // ROS_DEBUG_STREAM("constraints_upperbound_ size: " << constraints_upperbound_.size());
  // ROS_DEBUG_STREAM("circles: " << circles.size());
  // ROS_DEBUG_STREAM("ellipses: " << ellipses.size());
  // ROS_DEBUG_STREAM("vars size: " << vars_.size());

  // ROS_DEBUG_STREAM("vars_lowerbound_ size: " << vars_lowerbound_.size());
  // ROS_DEBUG_STREAM("vars_upperbound_ size: " << vars_upperbound_.size());
  // ROS_DEBUG_STREAM("Indices: indx: " << ind_x_ << " indy: " << ind_y_ << " indtheta: " << ind_theta_ << " indv: " << ind_v_ << " indw: " << ind_w_);
  // ROS_DEBUG_STREAM(" ind circles: " << ind_circles_ << "ind ellipses: " << ind_ellipses_ );

  publishResults();
}

void MPCNode::publishResults()
{
  succes_ = solution_.status;
  end_time_ = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_time = end_time_ - start_time_;

  ROS_DEBUG_STREAM("Solver succesful: " << succes_);
  ROS_DEBUG_STREAM("With constraint violations: " << fg_eval_.constr_viols_);
  ROS_DEBUG_STREAM("Elapsed time: " << elapsed_time.count());
  // ROS_DEBUG_STREAM("cost: " << solution_.obj_value);
  // ROS_DEBUG_STREAM("solution vector size: " << solution_.x.size());

  // Return the first actuator values
  double v_mpc = solution_.x[ind_v_];
  double w_mpc = solution_.x[ind_w_];

  if (succes_ && fg_eval_.constr_viols_ == false)
  {
    cmd_mpc_.linear.x = solution_.x[ind_v_];
    cmd_mpc_.angular.z = solution_.x[ind_w_];
  }
  else
  {
    // cmd_mpc_.linear.x = 0;
    // cmd_mpc_.angular.z = 0;
    cmd_mpc_.linear.x = v_user_;
    cmd_mpc_.angular.z = w_user_;
  }
  ROS_DEBUG_STREAM("Shared MPC Linear Velocity: " << cmd_mpc_.linear.x << " Angular Velocity: " << cmd_mpc_.angular.z);

  mpc_path_.poses.clear();
  mpc_path_.header = base_header_;
  // Visualization of predicted route
  for (size_t t = 0; t < Np_; t++)
  {
    mpc_path_.poses.push_back(rosmsg::makePoseStamped(solution_.x[ind_x_ + t],
                                                      solution_.x[ind_y_ + t],
                                                      solution_.x[ind_theta_ + t],
                                                      base_header_));
  }

  if (use_voronoi_)
  {
    if (voronoi_algorithm_.pub_markers_)
    {
      vorolines_publisher_.publish(voronoi_algorithm_.voronoi_lines_);
      delaunaylines_publisher_.publish(voronoi_algorithm_.delaunay_lines_);
    }

    voropath_publisher_.publish(voronoi_algorithm_.voro_path_);
  }

  user_path_publisher_.publish(user_path_);
  mpc_path_publisher_.publish(mpc_path_);
  cmd_publisher_.publish(cmd_mpc_);

  // ROS_DEBUG_STREAM("Succesfully published results");
}

void MPCNode::loadWarmStartPath(double v, double w)
{
  size_t t;
  ROS_DEBUG_STREAM("loading warm-start path with v = " << v << " w = " << w);

  // Apply warm start method: use previous solution, shifted by one time sample as new starting trajectory.
  for (t = 0; t < Np_ - 2; t++)
  {
    vars_[t + ind_v_] = solution_.x[ind_v_ + t + 1];
    vars_[t + ind_w_] = solution_.x[ind_w_ + t + 1];
    // vars_[t + ind_v_] = v;
    // vars_[t + ind_w_] = w;
  }
  for (t = 0; t < Np_ - 1; t++)
  {
    vars_[t + ind_x_] = solution_.x[ind_x_ + t + 1] - solution_.x[ind_x_ + 1];
    vars_[t + ind_y_] = solution_.x[ind_y_ + t + 1] - solution_.x[ind_y_ + 1];
    vars_[t + ind_theta_] = solution_.x[ind_theta_ + t + 1] - solution_.x[ind_theta_ + 1];
  }
  // extrapolate the last samples
  t = Np_ - 2;
  vars_[t + ind_v_] = vars_[t - 1 + ind_v_];
  vars_[t + ind_w_] = vars_[t - 1 + ind_w_];
  // vars_[t + ind_v_] = v;
  // vars_[t + ind_w_] = w;
  t = Np_ - 1;
  vars_[t + ind_x_] = vars_[t - 1 + ind_x_] + vars_[t - 1 + ind_v_] * cos(vars_[t - 1 + ind_theta_]) * time_interval_;
  vars_[t + ind_y_] = vars_[t - 1 + ind_y_] + vars_[t - 1 + ind_v_] * sin(vars_[t - 1 + ind_theta_]) * time_interval_;
  vars_[t + ind_theta_] = vars_[t - 1 + ind_theta_] + vars_[t - 1 + ind_w_] * time_interval_;
}

void MPCNode::loadPath(nav_msgs::Path path, double v, double w)
{
  // path in local coordinate frame
  ROS_DEBUG_STREAM("loading path with v = " << v << " w = " << w);
  size_t t;
  for (t = 0; t < Np_ - 1; t++)
  {
    vars_[t + ind_x_] = path.poses[t].pose.position.x;
    vars_[t + ind_y_] = path.poses[t].pose.position.y;
    vars_[t + ind_theta_] = tf::getYaw(path.poses[t].pose.orientation);
    vars_[t + ind_v_] = v;
    vars_[t + ind_w_] = w;
  }
  t = Np_ - 1;
  vars_[t + ind_x_] = path.poses[t].pose.position.x;
  vars_[t + ind_y_] = path.poses[t].pose.position.y;
  vars_[t + ind_theta_] = tf::getYaw(path.poses[t].pose.orientation);
}

void MPCNode::loadPath(std::vector<geometry_msgs::Pose2D> path, double v, double w)
{
  // path in local coordinate frame
  ROS_DEBUG_STREAM("loading path with v = " << v << " w = " << w);
  size_t t;
  for (t = 0; t < Np_ - 1; t++)
  {
    vars_[t + ind_x_] = path[t].x;
    vars_[t + ind_y_] = path[t].y;
    vars_[t + ind_theta_] = path[t].theta;
    vars_[t + ind_v_] = v;
    vars_[t + ind_w_] = w;
  }
  t = Np_ - 1;
  vars_[t + ind_x_] = path[t].x;
  vars_[t + ind_y_] = path[t].y;
  vars_[t + ind_theta_] = path[t].theta;
}

void MPCNode::processObstacles()
{
  // Clear the containers in which the obstacles will be loaded:
  // circles_ and ellipses_ are vectors each containing obstacles, each described by a vector of AD::double parameters
  circles_.clear();
  ellipses_.clear();
  // place obstacle message in voronoi object:
  if (use_voronoi_)
  {
    voronoi_algorithm_.obstacles_.objects.clear();
    voronoi_algorithm_.obstacles_.header = base_header_;
  }
  if (laser_obstacles_received_)
  {
    for (auto &object : laser_object_msg_.objects)
    {
      convertObstacleToCppad(object);
      if (use_voronoi_)
        voronoi_algorithm_.obstacles_.objects.push_back(object);
    }
  }
  if (cam_obstacles_received_)
  {
    for (auto &object : cam_object_msg_.objects)
    {
      convertObstacleToCppad(object);
      if (use_voronoi_)
        voronoi_algorithm_.obstacles_.objects.push_back(object);
    }
  }
}

void MPCNode::convertObstacleToCppad(const scat_msgs::EnvObject &object)
{
  // Add Walls, curbs and gaps (in the floor). All are described as lines.
  if (object.ID == 1 || object.ID == 2 || object.ID == 3)
  {
    addLineMPC(object.params[0], object.params[1], object.params[3], object.params[4]);
  }
  // Add Tables. described as polygons.
  else if (object.ID == 7)
  {
    addPolygonMPC(object.params);
  }
  // Add Humans. described as circles.
  else if (object.ID == 8)
  {
    circles_.push_back(addCircleMPC(object.params[0], object.params[1],
                                    object.params[3], object.params[4], object.params[5]));
  }
}

void MPCNode::addPolygonMPC(std::vector<float> polygon_params)
{
  int size = polygon_params.size();
  for (int i = 0; i < size; i += 3)
  {
    float &x1 = polygon_params[i];
    float &y1 = polygon_params[i + 1];
    float &x2 = polygon_params[(i + 3) % size];
    float &y2 = polygon_params[(i + 4) % size];
    addLineMPC(x1, y1, x2, y2);
  }
}

void MPCNode::addLineMPC(float x1, float y1, float x2, float y2)
{
  // Add line object interior point optimization constraints
  ellipses_.push_back(addEllipseMPC(x1, y1, x2, y2));

  if (use_corner_constraints_)
  {
    float vx = 0, vy = 0;
    if (pow(pow(x1, 2) + pow(y1, 2), 0.5) < obst_range_)
      circles_.push_back(addCircleMPC(x1, y1, 0, vx, vy));
    if (pow(pow(x2, 2) + pow(y2, 2), 0.5) < obst_range_)
      circles_.push_back(addCircleMPC(x2, y2, 0, vx, vy));
  }
}

CppAD::vector<AD<double>> MPCNode::addEllipseMPC(float x1, float y1, float x2, float y2)
{
  CppAD::vector<AD<double>> ellipse_data;
  ellipse_data.resize(5);
  AD<double> cx, cy, ctheta, fx, fy, f, a, b;
  // The axes of the ellipse are defined as follows:
  // The X axis is the short axis,
  // The Y axis aligns with the line obstacle
  // the angle of the ellipse is defined similar to ROS standard:
  // starting at ctheta = 0 at positive x-axis, with positive clockwise rotation.
  // So along the positive y-axis of the ellipsoid, ctheta = pi/2.
  fx = x2 - x1;
  fy = y2 - y1;
  f = sqrt(pow(fx, 2) + pow(fy, 2)) / 2;
  b = obst_dist_threshold_ + robot_radius_;
  a = sqrt(pow(f, 2) + pow(b, 2));
  cx = (x1 + x2) / 2;
  cy = (y1 + y2) / 2;
  ctheta = atan2(fy, fx) + M_PI;
  ellipse_data[0] = cx;
  ellipse_data[1] = cy;
  ellipse_data[2] = ctheta;
  ellipse_data[3] = a;
  ellipse_data[4] = b;
  return ellipse_data;
}

CppAD::vector<AD<double>> MPCNode::addCircleMPC(float x, float y, float r, float vx, float vy)
{
  CppAD::vector<AD<double>> circle_data;
  circle_data.resize(5);
  circle_data[0] = x;
  circle_data[1] = y;
  circle_data[2] = r + obst_dist_threshold_ + robot_radius_;
  circle_data[3] = vx;
  circle_data[4] = vy;
  return circle_data;
}

bool MPCNode::serviceCallback(std_srvs::SetBoolRequest &request,
                              std_srvs::SetBoolResponse &response)
{
  // This service can be used to activate / deactivate the node.
  node_active_ = request.data;
  response.success = true;
  ROS_WARN_STREAM("Shared MPC active: " << node_active_);
  return true;
}

} // namespace shared_mpc
