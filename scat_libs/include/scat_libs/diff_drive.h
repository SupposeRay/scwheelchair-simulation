#ifndef SCAT_DIFFDRIVE
#define SCAT_DIFFDRIVE
// ROS
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>
// C++
#include <stdlib.h>
#include <math.h>
#include <scat_libs/rosmsg.h>
#include <scat_libs/base_utils.h>

namespace diff_drive 
{
// The takeStep method takes a single time step, calculating the next position, using linear velocity v, angular velocity w, sampling time dt
void takeStep(geometry_msgs::Pose2D& pose, const float& v, const float& w, const float& dt);

// The following methods are different ways of extrapolating (predicting) the future path for a number of samples of a differential drive robot, based on current position and velocity. 
nav_msgs::Path makePath(double x, double y, double theta, const double& v, const double& w, const int& samples, const double& dt, const std_msgs::Header& header);
nav_msgs::Path makePath(double x, double y, double theta, const geometry_msgs::Twist& twist, const int& samples, const double& dt, const std_msgs::Header& header);
std::vector<geometry_msgs::Pose2D> makePath(const double& v, const double& w, const int& samples, const double& dt);
std::vector<geometry_msgs::Pose2D> makePath(const geometry_msgs::Twist& twist, const int& samples, const double& dt);
nav_msgs::Path makePath(const geometry_msgs::Twist& twist, const int& samples, const double& dt, const std_msgs::Header& header);
nav_msgs::Path makePath(const double& v, const double& w, const int& samples, const double& dt, const std_msgs::Header& header);

// The VelUtils object is used to limit velocity and acceleration to certain limits.
// The limits can be provided (usually once) by the setParams function.
// Then the limitVelocity or limitAccleration function can be called.
// Two overloaded methods are implemented for each function, with the same functionality but different interface (one using floats and another using ROS message types.)
class VelUtils
{
public:
    // Constructor
    VelUtils();
    // Destructor
    virtual ~VelUtils();

    void setParams(
        const float& v_min, const float& v_max,
        const float& v_max_neg,
        const float& w_min, const float& w_max,
        const float& v_acc, const float& v_dec,
        const float& w_acc, const float& w_dec,
        const float& v_threshold, const float& w_threshold,
        const float& sampling_time, const float& base_width);

    void limitVelocity(float &v, float &w);
    void limitAcceleration(float &v, float &w, const float &v_agent, const float &w_agent);
    geometry_msgs::Twist limitVelocity(const geometry_msgs::Twist &cmd_vel);
    geometry_msgs::Twist limitAcceleration(const geometry_msgs::Twist &cmd_vel,
                                           const geometry_msgs::Twist &vel_agent);

private:
    //! Maximum and Minimum linear velocities
    float v_max_ = 0.8, v_min_ = 0.35, v_max_neg_ = -0.7;
    //! Maximum and Minimum angular velocities
    float w_max_ = 1.8, w_min_ = 1.4; // [m/s] and [deg/s]
    // Maximum linear and angular acceleration
    float v_acc_ = 0.6, w_acc_ = 1.5; // m/s^2 and deg/s^2
    // Maximum brake accelarations, or 'Minimum' linear and angular acceleration
    float v_dec_ = 0.6, w_dec_ = 1.5; // m/s^2 and deg/s^2
    //! Activation thresholds
    float v_threshold_ = 0.1, w_threshold_ = 0.1;
    // time interval
    float dt_ = 0.2; // s
    // wheel gap used for differential drive calculations
    float base_width_ = 0.5;
};



} // namespace rosmsg

#endif