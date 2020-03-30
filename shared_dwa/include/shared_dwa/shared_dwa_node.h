// ROS
#include <ros/ros.h>
// tf
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include<sensor_msgs/point_cloud_conversion.h>
// Messages
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <scat_msgs/SharedVelocity.h>
#include <scat_msgs/EnvObjectList.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
// C++
#include <math.h>
#include <stdlib.h>
#include <thread>
#include <vector>
#include <typeinfo>
// Package
#include <scat_libs/base_utils.h>
#include <scat_libs/geom_utils.h>
#include <scat_libs/nav_utils.h>
#include <scat_libs/rosmsg.h>
#include <scat_libs/obst_dist.h>
#include <scat_libs/diff_drive.h>
#include <scat_libs/obst_utils.h>
#include <scat_msgs/EnvObjectList.h>
#include <scat_msgs/EnvObject.h>

namespace shared_dwa
{

#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)

class DWANode
{
public:
    // Constructor
    DWANode(ros::NodeHandle &node_handle);
    // Destructor
    virtual ~DWANode();

private:
    // Standard Node functions
    bool readParameters();
    bool readBotParameters();

    // Only one of the below callbacks will be used, depending on which value is set for obst_type: "scan", "obstmsg", "map", or "cloud"
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_scan);
    void obstCallback(const scat_msgs::EnvObjectList::ConstPtr &msg_obst);
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg_costmap);
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_cloud);

    void velCallback(const geometry_msgs::Twist::ConstPtr &msg_vel);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom);
    void userinputCallback(const geometry_msgs::Twist::ConstPtr &msg_userinput);
    void perfCallback(const std_msgs::Float32MultiArray::ConstPtr &msg_perf);
    void trajCallback(const nav_msgs::Path::ConstPtr &msg_traj);
    void timerCallback(const ros::TimerEvent &);
    void publishResults();
    bool serviceCallback(std_srvs::SetBoolRequest &request,
                         std_srvs::SetBoolResponse &response);

    // DWA functions
    void generateDynamicWindow();
    void selectVelocity();
    void predictTrajectory(float v_dw, float w_dw, float &size_path, 
                            float &clearance, float &safety, float &cost_ref_traj);
    float calcVelocity(float v_dw, float w_dw, float v_goal, float w_goal);
    float calcHeading(float v_dw, float w_dw, float theta_goal);
    float calcSmoothness(float v_dw, float w_dw);
    float calcRefTrajectory(float v_dw, float w_dw);
    float objFunction(float clearance, float cost_heading, float cost_velocity, float cost_smoothness, float cost_safety, float cost_ref_traj);

    //// State Containers
    sensor_msgs::PointCloud point_cloud_;
    nav_msgs::Path dwa_path_, user_path_;
    std::vector<geometry_msgs::Pose2D> sample_path_;
    visualization_msgs::Marker dwa_footprint_marker_, user_footprint_marker_;    
    visualization_msgs::Marker dynamic_window_marker_;
    std_msgs::Header header_;
    std_msgs::Float32MultiArray dwa_cost_;
    // Local reference trajectory which can be included in the cost function 
    nav_msgs::Path ref_trajectory_;
    // time variables
    ros::Time t_now_, t_prev_;
    ros::Duration dt_;
    // path_samples is used as a dynamic variable, equivalent to prediction horizon. However its size depends on the current velocity. 
    int path_samples_ = 10;


    //////// Variables ///////
    // current velocities from odometry
    float v_agent_ = 0, w_agent_ = 0;
    // current position
    float x_agent_ = 0, y_agent_ = 0, theta_agent_ = 0;
    // current velocities from userinput
    float v_user_ = 0, w_user_ = 0;
    // velocities selected by dwa algorithm
    float v_dwa_ = 0, w_dwa_ = 0;
    // float v_prev_ = 0, w_prev_ = 0;
    // angle of user input
    float theta_user_ = 0;
    // metric for general closeness to obstacles
    float av_danger_ = 0;
    // metric for average path length
    float av_path_length_ = 0;
    // metric giving the fraction of safe paths (no collision) relative to size of dynamic window
    float frac_safe_paths_ = 0;
    // Actual Dynamic Window values of translational and angular velocities
    std::vector<float> v_dw_vector_, w_dw_vector_;
    // Obstacles as a vector of vectors of points. Each vector of points describes a single obstacle. 
    std::vector<std::vector<geometry_msgs::Point>> obstacles_pts_;
    // Timing parameters for performance verification
    std::chrono::system_clock::time_point t0_,t1_,t2_;
    std::chrono::duration<double> dur1_, dur2_;

    //////// Standard Parameters ////////
    // bool values to check if the specific msg is received
    bool obst_received_ = false;
    bool vel_received_ = false;
    bool userinput_received_ = false;
    bool ref_trajectory_received_ = false;
    bool perf_received_ = false;
    bool use_adaptive_ = false;
    bool use_ref_trajectory_ = false;
    bool use_coll_margin_ = false;
    bool pub_shared_vel_ = false;
    bool node_active_ = false;
    //! method used for collision checking. 
    std::string collision_method_ = "footprint";
    // Reference frames
    std::string fixed_frame_id_ = "map", base_frame_id_ = "base_footprint";
    // vel type parameter setting
    std::string vel_type_ = "odom";
    // parameter determing which source is used for collision checking. can be "map", "scan", or "obstmsg"
    std::string obst_type_ = "scan"; 
    
    //////// DWA parameters ////////   
    // Period for position update of costmap reduction algorithm
    float pos_update_freq_ = 1.0;    
    // Constants currently being used in cost function
    float Kv_ = 0.3, Kh_ = 0.6, Kw_ = 0.0, Kc_ = 0.0, Ks_ = 0.1, Kref_ = 0.0;
    // Parameters defining minimum value and range of constants in cost function (iteratively adapted when in adaptive shared control mode)
    float Kv0_ = 0.15, Kh0_ = 0.35, Kw0_ = 0.0, Kvr_ = 0.2, Khr_ = 0.2, Kwr_ = 0.0;
    // max vel minus min vel
    float v_range_ = 1.5, w_range_ = 3.6;
    // parameter determing how much the footprint should be inflated.
    float footprint_inflation_ = 0.1; // 0.1 = 40%
    float vel_inflation_ = 0.4; // 0.4 = 40%. 
    // number of samples for linear and angular velocity
    int samples_v_ = 6;
    int samples_w_ = 8;
    float size_dw_ = 48;
    // Parameter, Number of samples in dwa
    int prediction_horizon_ = 6;
    // time interval of DWA
    float sampling_time_ = 0.4; // s      void camCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_cloud);

    // Threshold for costmap size reduction
    float occupancy_threshold_ = 50;
    // threshold for obstacle distance at which the point is considered to be in collision [m]
    float obst_inflation_ = 0.5;
    // Maximum distance at which obstacles are considered. Reducing this range improves calculation speed. 
    float obst_range_ = 10.0;
    //// Robot specific parameters
    //! Maximum linear and angular velocity
    float v_max_ = 0.8, v_min_ = 0.35, v_max_neg_ = -0.8, w_max_ = 1.8, w_min_ = 1.4; // [m/s] and [deg/s]
    // algorithm activation thresholds
    float v_threshold_ = 0.1, w_threshold_ = 0.1;
    // Maximum linear and angular acceleration
    float v_acc_ = 0.6, w_acc_ = 1.5; // m/s^2 and deg/s^2    
    // Maximum brake accelarations, or 'Minimum' linear and angular acceleration
    float v_dec_ = 0.6, w_dec_ = 1.5; // m/s^2 and deg/s^2    
    // largest distance from center of footprint to corner of footprint
    float L_max_ = 0.5;
    // Radius in [m]
    float robot_radius_ = 0.5;    
    // wheel gap used for differential drive calculations
    float base_width_ = 0.5;
    // threshold to avoid taking the floor into account. Any points in the point cloud, with height above this threshold, will be checked for obstacle avoidance in the collision check. 
    float height_threshold_ = 0.3;    
    //! footprint data
    std::vector<geometry_msgs::Point> footprint_;
    std::vector<float> footprint_coords_;

    //// ROS objects
    ros::NodeHandle &node_handle_;
    ros::ServiceServer activation_service_;
    ros::Timer timer_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tf_listener_;
    ros::Subscriber obst_subscriber_, vel_subscriber_, odom_subscriber_,
        userinput_subscriber_, userperf_subscriber_, position_subscriber_, ref_traj_subscriber_;
    ros::Publisher cmd_publisher_, diff_cmd_publisher_, dw_marker_publisher_, cost_publisher_,
        dwa_path_publisher_, user_path_publisher_, dwa_shape_publisher_, user_shape_publisher_;

    // Obstacle Distance calculation object from scat_libs
    obst_dist::ObstacleDistance obst_dist_;
    
    // Vel Utils object to calculate the limited velocity
    diff_drive::VelUtils vel_utils_;
};
} // namespace shared_dwa
