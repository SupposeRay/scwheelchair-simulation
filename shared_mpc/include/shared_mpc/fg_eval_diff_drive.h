// ROS
#include <ros/ros.h>
// Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
// C++
#include <stdlib.h>
#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Dense>
// Package
#include <laser_line_extraction/LineSegmentList.h>
#include <laser_line_extraction/LineSegment.h>

namespace shared_mpc
{
using CppAD::AD;

class FG_eval
{
public:
  FG_eval();
  ~FG_eval();
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void setParams(std::vector<size_t> &indices,
                 std::vector<double> &constants,
                 double &v_max, 
                 double &w_max,
                 size_t &Np,
                 double &dt,
                 size_t &n_vars);

  void update(std::vector<double> &U0,
              std::vector<double> &U_ref,
              size_t &n_constraints,
              size_t &ind_circles,
              size_t &ind_ellipses,
              CppAD::vector<CppAD::vector<AD<double>>> &circles,
              CppAD::vector<CppAD::vector<AD<double>>> &ellipses);

  void operator()(ADvector &fg, const ADvector &vars);

  // Cost function constants (velocity, heading, clearance, smoothness)
  AD<double> Kv_, Kw_, Kh_, Ks_, Keq_, Kineq_;
  // boolean indicating whether there are any constraint violations.
  bool constr_viols_ = true;

private:
  //// Variables
  // indices of states and inputs
  size_t ind_x_, ind_y_, ind_theta_, ind_v_, ind_w_, ind_circles_, ind_ellipses_;
  // number of vars and constraints;
  size_t n_vars_, n_constraints_;
  // reference velocities
  AD<double> v_ref_, w_ref_;
  // current agent velocity
  AD<double> v_agent_, w_agent_;
  // velocity range parameter
  AD<double> v_range_, w_range_;
  AD<double> heading_user_;
  // obstacles
  CppAD::vector<CppAD::vector<AD<double>>> circles_;
  CppAD::vector<CppAD::vector<AD<double>>> ellipses_;
    
  //// Parameters
  // Prediction Horizon
  size_t Np_ = 6;
  AD<double> Np_d_ = 6;
  // Sampling time
  AD<double> dt_ = 0.4;
  // Agent position
  AD<double> x_agent_, y_agent_, theta_agent_;
  // Max/min velocities
  AD<double> v_max_ = 0.8, w_max_ = 1.8;
};

} // namespace shared_mpc
