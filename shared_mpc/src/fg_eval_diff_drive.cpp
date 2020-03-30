#include "shared_mpc/fg_eval_diff_drive.h"

namespace shared_mpc
{
// Constructor
FG_eval::FG_eval() {}

// Destructor
FG_eval::~FG_eval() {}

void FG_eval::setParams(std::vector<size_t> &indices,
                        std::vector<double> &constants,
                        double& v_max, 
                        double& w_max,
                        size_t &Np,
                        double &dt,
                        size_t &n_vars)
{
  ind_x_ = indices[0];
  ind_y_ = indices[1];
  ind_theta_ = indices[2];
  ind_v_ = indices[3];
  ind_w_ = indices[4];
    Kv_ = constants[0];
  Kw_ = constants[1];
  Kh_ = constants[2];
  Ks_ = constants[3];
  Keq_ = constants[4];
  Kineq_ = constants[5];
  v_max_ = v_max;
  w_max_ = w_max;
  Np_ = Np;
  Np_d_ = static_cast<double>(Np);
  dt_ = dt;
  n_vars_ = n_vars;
}

void FG_eval::update(std::vector<double> &U0,
                     std::vector<double> &U_ref,
                     size_t &n_constraints,
                     size_t &ind_circles,
                     size_t &ind_ellipses,
                     CppAD::vector<CppAD::vector<AD<double>>> &circles,
                     CppAD::vector<CppAD::vector<AD<double>>> &ellipses)
{
  ROS_DEBUG_STREAM("Updating fg_eval");
  v_agent_ = U0[0];
  w_agent_ = U0[1];
  v_ref_ = U_ref[0];
  w_ref_ = U_ref[1];
  n_constraints_ = n_constraints;  
  ind_circles_ = ind_circles;  
  circles_.clear();
  circles_ = circles;    
  ind_ellipses_ = ind_ellipses;  
  ellipses_.clear();
  ellipses_ = ellipses;  
  heading_user_ = CppAD::atan2(w_ref_/w_max_, v_ref_/v_max_);;
}

void FG_eval::operator()(ADvector &fg, const ADvector &vars)
{
  ROS_DEBUG_STREAM("Calling cost function");
  ROS_DEBUG_STREAM("v_ref: " << v_ref_ << " w_ref: " << w_ref_);

  assert(fg.size() == 1+n_constraints_);              
  assert(vars.size() == n_vars_); 
  constr_viols_ = false;
  // Initialize cost to 0. Total calculated cost is located at fg[0]
  fg[0] = 0;

  // //// Readable obedience cost calculation
  // for (size_t t = 0; t < Np_ - 1; t++)
  // {
  //   AD<double> v = vars[t + ind_v_];
  //   AD<double> w = vars[t + ind_w_]; //

  //   // Velocity
  //   // velocity_cost = Kv_ * CppAD::pow(v - v_ref_, 2) + Kw_ * CppAD::pow(w - w_ref_, 2);
  //   fg[0] += Kv_ * CppAD::pow((v - v_ref_)/v_max_, 2); 
  //   // fg[0] += Kw_ * CppAD::pow((w - w_ref_)/w_max_, 2);

  //   // Heading
  //   AD<double> heading_mpc = CppAD::atan2(w/w_max_, v/v_max_ + 0.01);
  //   fg[0] += Kh_ * CppAD::pow((heading_user_ - heading_mpc) / 2*M_PI, 2 );
  //   // ROS_INFO_STREAM("v_ref_: " << v_ref_);
  //   // ROS_INFO_STREAM("w_ref_: " << w_ref_);      
  //   // ROS_INFO_STREAM("v: " << v);
  //   // ROS_INFO_STREAM("w: " << w);
  //   // ROS_INFO_STREAM("heading_user_: " << heading_user_);
  //   // ROS_INFO_STREAM("heading_mpc: " << heading_mpc);
  //   // ROS_INFO_STREAM("cost: " <<  Kh_ * CppAD::pow((heading_user_ - heading_mpc) / 2*M_PI, 2 ));
  // }

  //// Optimized obedience cost calculation
  for (size_t t = 0; t < Np_ - 1; t++)
  {
    // Velocity
    fg[0] += Kv_ * CppAD::pow((vars[t + ind_v_] - v_ref_)/v_max_, 2); 
    // Heading
    fg[0] += Kh_ * CppAD::pow((heading_user_ -  CppAD::atan2( vars[t + ind_w_]/w_max_, 
                                                              vars[t + ind_v_]/v_max_ + 0.01)) / 2*M_PI, 2 );
  }

  // cost based on actuation smoothness
  // fg[0] += Np_d_ * Ks_ * (CppAD::pow( (v_agent_ - vars[ind_v_])/v_max_, 2) + 
  //                         CppAD::pow( (w_agent_ - vars[ind_w_])/w_max_, 2) );     
  for (size_t t = 0; t < Np_ - 2; t++)
  {
    //// Readable smoothness cost calculation
    // AD<double> v0 = vars[t + ind_v_] / v_max_;
    // AD<double> w0 = vars[t + ind_w_] / w_max_;
    // AD<double> v1 = vars[t + 1 + ind_v_] / v_max_;
    // AD<double> w1 = vars[t + 1 + ind_w_] / w_max_;

    // // Smoothness
    // // smoothness_cost = CppAD::pow(v1 - v0, 2) + CppAD::pow(w1 - w0, 2);
    // fg[0] += Ks_ * (CppAD::pow(v1 - v0, 2) + CppAD::pow(w1 - w0, 2));

    //// Optimized smoothness cost calculation
    fg[0] += Ks_ * (CppAD::pow( (vars[t + 1 + ind_v_] - vars[t + ind_v_])/ v_max_, 2) + 
                    CppAD::pow( (vars[t + 1 + ind_w_] - vars[t + ind_w_])/w_max_, 2) );              
  }

  // Since the cost is located at index 0 of fg, skip index by +1
  // fg[1 + ind_x_] = vars[ind_x_];
  // fg[1 + ind_y_] = vars[ind_y_];
  // fg[1 + ind_theta_] = vars[ind_theta_];
  // Constraints based on error compared to system model
  // AD<double> time = 0;
  for (size_t t = 0; t < Np_ - 1; t++)
  {
    //// Readable dynamic model constraint calculation
    // // State at time t
    // AD<double> x0 = vars[t + ind_x_];
    // AD<double> y0 = vars[t + ind_y_];
    // AD<double> theta0 = vars[t + ind_theta_];

    // // State at time t+1
    // AD<double> x1 = vars[t + 1 + ind_x_];
    // AD<double> y1 = vars[t + 1 + ind_y_];
    // AD<double> theta1 = vars[t + 1 + ind_theta_];

    // // Actuation at time t.
    // AD<double> v0 = vars[t + ind_v_];
    // AD<double> w0 = vars[t + ind_w_];

    // // Calculate model constraints at time t+1, skip index by +1
    // fg[t + 2 + ind_x_] = Keq_ * (x1 - (x0 + v0 * CppAD::cos(theta0) * dt_));
    // fg[t + 2 + ind_y_] = Keq_ * (y1 - (y0 + v0 * CppAD::sin(theta0) * dt_));
    // fg[t + 2 + ind_theta_] = Keq_ * (theta1 - (theta0 + w0 * dt_));

    //// Optimized dynamic model constraint calculation
    fg[t + 2 + ind_x_] = Keq_ * CppAD::pow( (vars[t + 1 + ind_x_] - (vars[t + ind_x_] + vars[t + ind_v_] * CppAD::cos(vars[t + ind_theta_]) * dt_)), 2);
    fg[t + 2 + ind_y_] = Keq_ * CppAD::pow((vars[t + 1 + ind_y_] - (vars[t + ind_y_] + vars[t + ind_v_] * CppAD::sin(vars[t + ind_theta_]) * dt_)), 2);
    fg[t + 2 + ind_theta_] = Keq_ * CppAD::pow((vars[t + 1 + ind_theta_] - (vars[t + ind_theta_] +  vars[t + ind_w_] * dt_)), 2);

    // //// circle constraint
    for (int i = 0; i < circles_.size(); i++)
    {
      //// Readable circle constraint calculation
    //   AD<double> vx = circles_[i][3];
    //   AD<double> vy = circles_[i][4];      
    //   AD<double> cx = circles_[i][0] + vx*time*dt_;
    //   AD<double> cy = circles_[i][1] + vx*time*dt_;
    //   AD<double> radius = circles_[i][2];
    //   AD<double> distance = CppAD::pow((CppAD::pow((cx - x1), 2) 
    //                                   + CppAD::pow((cy - y1), 2)), 0.5);
    //   AD<double> constraint = CppAD::pow(radius - distance,2);
    //   fg[t + 1 + ind_circles_ + i * Np_] = Kineq_ * constraint;

      //// Optimized circle constraint calculation
      fg[t + 1 + ind_circles_ + i * Np_] = Kineq_ * (circles_[i][2] - 
                                                      CppAD::pow(
                                                          CppAD::pow((circles_[i][0] - vars[t + 1 + ind_x_]), 2) 
                                                        + CppAD::pow((circles_[i][1] - vars[t + 1 + ind_y_]), 2), 0.5));
    //   if (constraint>0)
    //   {
    //     // ROS_WARN_STREAM("time: " << t );
    //     // ROS_WARN_STREAM("distance path to circle: " << distance);
    //     // ROS_WARN_STREAM("step distance: " << pow((pow((x1-x0),2) + pow((y1-y0),2)),0.5));
    //     // ROS_WARN_STREAM("circle: " << i );
    //     constr_viols_ = true;
    //   }
    }  

    //// ellipse constraint
    for (int i = 0; i < ellipses_.size(); i++)
    {
      //// Readable ellipse constraint calculation
      // AD<double> cx = ellipses_[i][0];
      // AD<double> cy = ellipses_[i][1];
      // AD<double> ctheta = ellipses_[i][2];
      // AD<double> a = ellipses_[i][3];
      // AD<double> b = ellipses_[i][4];
      // // transform coordinates of waypoint into ellipse frame of reference
      // AD<double> x_ellipse =  (x1 - cx) * CppAD::cos(ctheta) + (y1 - cy) * CppAD::sin(ctheta);
      // AD<double> y_ellipse = -(x1 - cx) * CppAD::sin(ctheta) + (y1 - cy) * CppAD::cos(ctheta);
      // // characteristic ellipse equation
      // AD<double> constraint = 1 - (CppAD::pow((x_ellipse),2)/CppAD::pow(a,2) 
      //                           + CppAD::pow((y_ellipse),2)/CppAD::pow(b,2));                                                                              
      // fg[t + 1 + ind_ellipses_ + i * Np_] = Kineq_*constraint;

      //// Optimized ellipse constraint calculation
      fg[t + 1 + ind_ellipses_ + i * Np_] = Kineq_* (1 - 
                                            (CppAD::pow(((vars[t + 1 + ind_x_] - ellipses_[i][0]) * CppAD::cos(ellipses_[i][2]) + (vars[t + 1 + ind_y_] - ellipses_[i][1]) * CppAD::sin(ellipses_[i][2])),2)/CppAD::pow(ellipses_[i][3],2) 
                                           + CppAD::pow((-(vars[t + 1 + ind_x_] - ellipses_[i][0]) * CppAD::sin(ellipses_[i][2]) + (vars[t + 1 + ind_y_] - ellipses_[i][1]) * CppAD::cos(ellipses_[i][2])),2)/CppAD::pow(ellipses_[i][4],2)));

      // if (constraint>0)
      // {
      //   // ROS_WARN_STREAM("time: " << t );
      //   // ROS_WARN_STREAM("distance path to ellipse: " << (1 - constraint));
      //   // ROS_WARN_STREAM("step distance: " << pow((pow((x1-x0),2) + pow((y1-y0),2)),0.5));
      //   // ROS_WARN_STREAM("x: " << x1 << "y: " << y1 );
      //   // ROS_WARN_STREAM("ellipse: " << i );
      //   // ROS_WARN_STREAM("cx: " << cx << "cy: " << cy );        
      //   constr_viols_ = true;
      // }
      // time += 1;
    }  


  }

  return;
}
} // namespace shared_mpc
