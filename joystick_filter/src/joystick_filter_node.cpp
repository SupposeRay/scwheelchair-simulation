#include "joystick_filter/joystick_filter_node.h"

namespace joystick_filter_node
{

FilterNode::FilterNode(ros::NodeHandle &node_handle)
    : node_handle_(node_handle)
{
  ROS_DEBUG("Launching Constructor");

  if (!readParameters())
  {
    ROS_ERROR("Could not load parameters.");
    ros::requestShutdown();
  }
  t_prev_ = ros::Time::now();

  initializeMatrices();

  // Subscribers
  userinput_subscriber_ = node_handle_.subscribe("/user/joy", 1, &FilterNode::userinputCallback, this);

  // Publishers
  userinput_publisher_ = node_handle_.advertise<std_msgs::Float32MultiArray>("/user/joy/filtered", 1);
  usercmd_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/user/cmd_vel", 1);

  performance_publisher_ = node_handle_.advertise<std_msgs::Float32MultiArray>("/user/performance", 1);

  ROS_DEBUG("Succesfully started Joystick Filter Node.");
}

FilterNode::~FilterNode() {}

bool FilterNode::readParameters()
{
  bool debug = false;
  if (!node_handle_.getParam("debug", debug))
    ROS_WARN_STREAM("Parameter debug not set for sparse_voronoi. Using default setting: " << debug);      
  if (!node_handle_.getParam("publish_frequency", publish_frequency_))
    ROS_WARN_STREAM("Parameter publish_frequency not set for joystick_filter. Using default setting: " << publish_frequency_);
  if (!node_handle_.getParam("use_kalmannfilter", use_kalmannfilter_))
    ROS_WARN_STREAM("Parameter use_kalmannfilter not set for joystick_filter. Using default setting: " << use_kalmannfilter_);
  if (!node_handle_.getParam("adaptive_kalman", adaptive_kalman_))
    ROS_WARN_STREAM("Parameter adaptive_kalman not set for joystick_filter. Using default setting: " << adaptive_kalman_);
  if (!node_handle_.getParam("use_medianfilter", use_medianfilter_))
    ROS_WARN_STREAM("Parameter use_medianfilter not set for joystick_filter. Using default setting: " << use_medianfilter_);
  if (!node_handle_.getParam("kv", k_v_))
    ROS_WARN_STREAM("Parameter kv not set for joystick_filter. Using default setting: " << k_v_);
  if (!node_handle_.getParam("kw", k_w_))
    ROS_WARN_STREAM("Parameter kw not set for joystick_filter. Using default setting: " << k_w_);
  if (!node_handle_.getParam("deadzone", deadzone_))
    ROS_WARN_STREAM("Parameter deadzone not set for joystick_filter. Using default setting: " << deadzone_);
  if (!node_handle_.getParam("control_exponent", control_exp_))
    ROS_WARN_STREAM("Parameter control_exponent not set for joystick_filter. Using default setting: " << control_exp_);
  if (!node_handle_.getParam("buffer_size", buffer_size_))
    ROS_WARN_STREAM("Parameter buffer_size not set for joystick_filter. Using default setting: " << buffer_size_);
  if (!node_handle_.getParam("P0", P0_))
    ROS_WARN_STREAM("Parameter P0 not set for joystick_filter. Using default setting: " << P0_);
  if (!node_handle_.getParam("cov_x", cov_x_))
    ROS_WARN_STREAM("Parameter cov_x not set for joystick_filter. Using default setting: " << cov_x_);
  if (!node_handle_.getParam("cov_y", cov_y_))
    ROS_WARN_STREAM("Parameter cov_y not set for joystick_filter. Using default setting: " << cov_y_);
  if (!node_handle_.getParam("cov_vx", cov_vx_))
    ROS_WARN_STREAM("Parameter cov_vx not set for joystick_filter. Using default setting: " << cov_vx_);
  if (!node_handle_.getParam("cov_vy", cov_vy_))
    ROS_WARN_STREAM("Parameter cov_vy not set for joystick_filter. Using default setting: " << cov_vy_);
  if (!node_handle_.getParam("Qx", Qx_))
    ROS_WARN_STREAM("Parameter Qx not set for joystick_filter. Using default setting: " << Qx_);
  if (!node_handle_.getParam("Qy", Qy_))
    ROS_WARN_STREAM("Parameter Qy not set for joystick_filter. Using default setting: " << Qy_);
  if (!node_handle_.getParam("Hx", Hx_))
    ROS_WARN_STREAM("Parameter Hx not set for joystick_filter. Using default setting: " << Hx_);
  if (!node_handle_.getParam("Hy", Hy_))
    ROS_WARN_STREAM("Parameter Hy not set for joystick_filter. Using default setting: " << Hy_);
  if (!node_handle_.getParam("alpha", alpha_))
    ROS_WARN_STREAM("Parameter alpha not set for joystick_filter. Using default setting: " << alpha_);
  if (!node_handle_.getParam("display_variance_matrices", display_variance_matrices_))
    ROS_WARN_STREAM("Parameter display_variance_matrices not set for joystick_filter. Using default setting: " << display_variance_matrices_);

  // When debugging, set logger_level to Debug by changing Info to Debug.
  if (debug)
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  else
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
      
  ROS_DEBUG("Parameters read.");

  if (adaptive_kalman_)
    ROS_INFO_STREAM("Joystick filter is using adaptive kalman filter. Publishing joystick performance parameters.");

  return true;
}

void FilterNode::initializeMatrices()
{
  ROS_DEBUG("Initializing Matrices.");
  // initialize state variable: X = x, y, xdot, ydot
  X_ << 0, 0, 0, 0;

  // Initialize State Transition Function: later will be updated with each iteration, by measuring dt.
  float dt = 1 / publish_frequency_;
  F_ << 1, 0, dt, 0,
      0, 1, 0, dt,
      0, 0, 1, 0,
      0, 0, 0, 1;
  // Initialize uncertainty
  P_ << P0_, 0, 0, 0,
      0, P0_, 0, 0,
      0, 0, P0_, 0,
      0, 0, 0, P0_;
  // Initialize Measurement Function
  H_ << 1, 0, Hx_ * dt, 0,
      0, 1, 0, Hy_ * dt,
      0, 0, 0, 0,
      0, 0, 0, 0;
  // Initialize Process Covariance Matrix (action uncertainty)
  Q_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, Qx_, 0,
      0, 0, 0, Qy_;
  // Initialize Measurement Covariance Matrix (measurement uncertainty)
  R_ << cov_x_, 0, 0, 0,
      0, cov_y_, 0, 0,
      0, 0, cov_vx_, 0,
      0, 0, 0, cov_vy_;
  // Initialize state measurement
  Z_ << 0, 0, 0, 0;
  // Initialize state measurement innovation
  Y_ << 0, 0, 0, 0;
  // Initialize state prediction residual (epsilon)
  eps_ << 0, 0, 0, 0;
  // Initialize Measurement noise matrix
  S_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;
  // Initialize Kalman gain matrix
  K_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;
}
void FilterNode::resetBuffer()
{
  buffer_x_.clear();
  buffer_y_.clear();
}  
void FilterNode::resetMatrices()
{
  // reset state variable: X = x, y, xdot, ydot
  X_ << 0, 0, 0, 0;
  ux_ = 0;
  uy_ = 0;
  vx_ = 0;
  vy_ = 0;
  // reset State Transition Function: later will be updated with each iteration, by measuring dt.
  float dt = 1 / publish_frequency_;
  F_ << 1, 0, dt, 0,
      0, 1, 0, dt,
      0, 0, 1, 0,
      0, 0, 0, 1;
  // reset uncertainty
  P_ << P0_, 0, 0, 0,
      0, P0_, 0, 0,
      0, 0, P0_, 0,
      0, 0, 0, P0_;
  // reset Measurement Function
  H_ << 1, 0, Hx_ * dt, 0,
      0, 1, 0, Hy_ * dt,
      0, 0, 0, 0,
      0, 0, 0, 0;
  // reset Process Covariance Matrix (action uncertainty)
  Q_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, Qx_, 0,
      0, 0, 0, Qy_;
  // reset Measurement Covariance Matrix (measurement uncertainty)
  R_ << cov_x_, 0, 0, 0,
      0, cov_y_, 0, 0,
      0, 0, cov_vx_, 0,
      0, 0, 0, cov_vy_;
  // reset state measurement
  Z_ << 0, 0, 0, 0;
  // reset state measurement innovation
  Y_ << 0, 0, 0, 0;
  // reset state prediction residual (epsilon)
  eps_ << 0, 0, 0, 0;
  // reset Measurement noise matrix
  S_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;
  // reset Kalman gain matrix
  K_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;
}

void FilterNode::userinputCallback(const std_msgs::Float32MultiArray &msg_userinput)
{
  // Update time measurement
  t_ = ros::Time::now();
  ros::Duration dt_ros = t_ - t_prev_;
  dt_ = dt_ros.toSec();
  if (dt_ > 1.0 / publish_frequency_)
  {
    // New user commands are processed according to the publish frequency, to avoid unnecessary calculations
    t_prev_ = t_;
    // check input for nans:
    if (  (fabs(msg_userinput.data[0]) < deadzone_ && fabs(msg_userinput.data[1]) < deadzone_) 
          || std::isnan(msg_userinput.data[0]) || std::isnan(msg_userinput.data[1]) )
    {
      resetMatrices();
      resetBuffer();
      std::vector<float> cmd_array = {0, 0};
      publishResults(rosmsg::makeFloat32MultiArray(cmd_array));
    }
    else
    {
      std_msgs::Float32MultiArray userinput = msg_userinput;
      if ((userinput.data[0] < 0.0 && ux_ > 0.0) || (userinput.data[0] > 0.0 && ux_ < 0.0)
          || (userinput.data[1] < 0.0 && uy_ > 0.0) || (userinput.data[1] > 0.0 && uy_ < 0.0))
        resetMatrices();
       
      if (use_medianfilter_)
        medianFilter(userinput); // to filter out sensor noise
      if (use_kalmannfilter_)
        kalmanFilter(userinput); // Adaptive filtering

      expControlFunction(userinput);
      deadZone(userinput);
      publishResults(userinput);
    }
  }
}

void FilterNode::publishResults(const std_msgs::Float32MultiArray &userinput)
{
  // Store filtered data as cmd_vel Twist message
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = k_v_ * userinput.data[0];
  cmd_vel.angular.z = k_w_ * userinput.data[1];
  // cmd_vel.angular.z = base_utils::sign(userinput.data[0]) * k_w_ * userinput.data[1]; // inverse rotation when going backwards
  // Publish results
  userinput_publisher_.publish(userinput); // filtered user input
  usercmd_publisher_.publish(cmd_vel);     // velocity commands
  // Publish Performance results
  std::vector<float> performance = {R_(2, 2), R_(3, 3), R_(2, 3)};
  performance_publisher_.publish(rosmsg::makeFloat32MultiArray(performance));

  if (display_variance_matrices_)
  {
    ROS_DEBUG_STREAM("Process covariance matrix Q");
    ROS_DEBUG_STREAM(Q_(0,0) << " , " << Q_(0,1) << " , " << Q_(0,2) << " , " << Q_(0,3));
    ROS_DEBUG_STREAM(Q_(1,0) << " , " << Q_(1,1) << " , " << Q_(1,2) << " , " << Q_(1,3));
    ROS_DEBUG_STREAM(Q_(2,0) << " , " << Q_(2,1) << " , " << Q_(2,2) << " , " << Q_(2,3));
    ROS_DEBUG_STREAM(Q_(3,0) << " , " << Q_(3,1) << " , " << Q_(3,2) << " , " << Q_(3,3));

    ROS_DEBUG_STREAM("Measurement covariance matrix R");
    ROS_DEBUG_STREAM(R_(0,0) << " , " << R_(0,1) << " , " << R_(0,2) << " , " << R_(0,3));
    ROS_DEBUG_STREAM(R_(1,0) << " , " << R_(1,1) << " , " << R_(1,2) << " , " << R_(1,3));
    ROS_DEBUG_STREAM(R_(2,0) << " , " << R_(2,1) << " , " << R_(2,2) << " , " << R_(2,3));
    ROS_DEBUG_STREAM(R_(3,0) << " , " << R_(3,1) << " , " << R_(3,2) << " , " << R_(3,3));    
  }
}

void FilterNode::deadZone(std_msgs::Float32MultiArray &userinput)
{
  if (sqrt(pow(userinput.data[0],2) + pow(userinput.data[1],2)) < deadzone_)
  {
    userinput.data[0] = 0;
    userinput.data[1] = 0;
  }
}

void FilterNode::expControlFunction(std_msgs::Float32MultiArray &userinput)
{
  userinput.data[0] = base_utils::sign(userinput.data[0]) * pow(fabs(userinput.data[0]), control_exp_);
  userinput.data[1] = base_utils::sign(userinput.data[1]) * pow(fabs(userinput.data[1]), control_exp_);
}

void FilterNode::kalmanFilter(std_msgs::Float32MultiArray &userinput)
{
  // Update measurement
  vx_ = (userinput.data[0] - ux_) / dt_;
  vy_ = (userinput.data[1] - uy_) / dt_;
  ux_ = userinput.data[0];
  uy_ = userinput.data[1];
  Z_ << ux_, uy_, vx_, vy_;
  // Update state transition matrix
  F_ << 1, 0, dt_, 0,
      0, 1, 0, dt_,
      0, 0, 1, 0,
      0, 0, 0, 1;
  // // Update measurement matrix
  H_ << 1, 0, Hx_ * dt_, 0,
      0, 1, 0, Hy_ * dt_,
      0, 0, 0, 0,
      0, 0, 0, 0;

  // PREDICT current state based on previous state and transition model.
  kalmanPredict();

  // UPDATE current state based on state prediction and new measurement
  kalmanUpdate();

  // return data
  userinput.data[0] = X_(0);
  userinput.data[1] = X_(1);
}

void FilterNode::kalmanPredict()
{
  // STATE PREDICTION
  // Predict new state
  X_ = F_ * X_;
  // Predict new state covariance matrix
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void FilterNode::kalmanUpdate()
{
  // MEASUREMENT UPDATE
  // Calculate measurement residual Y =  difference between measurement and predicted measurement.
  Y_ = Z_ - H_ * X_;
  // Calculate measurement noise S = Covariance of Y = E[Y^T * Y] (used for Kalman gain calculation)
  S_ = H_ * P_ * H_.transpose() + R_;

  // POSTERIOR UPDATE
  // Calculate Kalman gain based on Covariance (uncertainty).
  // This matrix determines how much weight is placed on previous state and new measurement
  K_ = P_ * H_.transpose() * S_.inverse();
  // Update state vector
  X_ = X_ + K_ * Y_;

  // Adaptive covariance matrix
  // Calculate residual error = difference between measurement and posteriori estimated state
  eps_ = Z_ - H_ * X_;
  // Residual based measurement covariance estimation: difference between posteriori residual error of state and priori state covariance (expected noise)
  Ra_ = alpha_ * R_ + (1 - alpha_) * (eps_ * eps_.transpose() + H_ * P_ * H_.transpose());
// Innovation based Adaptive estimation of Q (based on priori estimate)
  Qa_ = alpha_ * Q_ + (1 - alpha_) * (K_ * Y_ * Y_.transpose() * K_.transpose());

  if (adaptive_kalman_)
  {
    R_ = Ra_;
    Q_ = Qa_;
  }

  // Update state covariance matrix
  P_ = (I_ - K_ * H_) * P_;
}

void FilterNode::medianFilter(std_msgs::Float32MultiArray &userinput)
{
  buffer_x_.push_back(userinput.data[0]);
  buffer_y_.push_back(userinput.data[1]);
  if (buffer_x_.size() > buffer_size_)
  {
    buffer_x_.pop_front();
    buffer_y_.pop_front();
  }
  average_x_ = std::accumulate(buffer_x_.begin(), buffer_x_.end(), 0.0) / buffer_x_.size();
  average_y_ = std::accumulate(buffer_y_.begin(), buffer_y_.end(), 0.0) / buffer_y_.size();

  userinput.data[0] = average_x_;
  userinput.data[1] = average_y_;
}

} // namespace joystick_filter_node
