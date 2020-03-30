#include "shared_dwa/shared_dwa_node.h"

namespace shared_dwa
{
// Constructor
DWANode::DWANode(ros::NodeHandle &node_handle)
    : node_handle_(node_handle)
{

    ROS_DEBUG("Launching Shared DWA");

    if (!readParameters() || !readBotParameters())
    {
        ROS_ERROR("Could not load parameters.");
        ros::requestShutdown();
    }

    // Set parameters for the velocity limiter object.
    vel_utils_.setParams(
        v_min_, v_max_,
        v_max_neg_,
        w_min_, w_max_,
        v_acc_, v_dec_,
        w_acc_, w_dec_,
        v_threshold_, w_threshold_,
        sampling_time_, base_width_);

    // Subscribers & Publishers
    userinput_subscriber_ = node_handle_.subscribe("/user/cmd_vel", 1, &DWANode::userinputCallback, this);
    // Can use either map and/or scan data for obstacle avoidance (also both).
    // Map is only used when parameter collision_method == radius.
    if (obst_type_ == "map")
        obst_subscriber_ = node_handle_.subscribe("/move_base/local_costmap/costmap", 1, &DWANode::costmapCallback, this);
    else if (obst_type_ == "scan")
        obst_subscriber_ = node_handle_.subscribe("/scan", 1, &DWANode::scanCallback, this);
    else if (obst_type_ == "obstmsg")
        obst_subscriber_ = node_handle_.subscribe("/obstacles", 1, &DWANode::obstCallback, this);
    else if (obst_type_ == "cloud")
        obst_subscriber_ = node_handle_.subscribe("/pointcloud", 1, &DWANode::cloudCallback, this);

    if (vel_type_ == "odom")
        odom_subscriber_ = node_handle_.subscribe("/odom", 1, &DWANode::odomCallback, this);
    else if (vel_type_ == "twist")
        vel_subscriber_ = node_handle_.subscribe("/velocity", 1, &DWANode::velCallback, this);

    if (use_adaptive_)
        userperf_subscriber_ = node_handle_.subscribe("/user/performance", 1, &DWANode::perfCallback, this);

    if (use_ref_trajectory_)
        ref_traj_subscriber_ = node_handle_.subscribe("/sparse_voronoi/interp_path", 1, &DWANode::trajCallback, this);

    cmd_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    timer_ = node_handle_.createTimer(ros::Duration(pos_update_freq_), &DWANode::timerCallback, this);
    activation_service_ = node_handle_.advertiseService("activation_service", &DWANode::serviceCallback, this);

    if (pub_shared_vel_)
        diff_cmd_publisher_ = node_handle_.advertise<scat_msgs::SharedVelocity>("/shared_vel", 1);

    dw_marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("dynamic_window", 1);
    dwa_path_publisher_ = node_handle_.advertise<nav_msgs::Path>("dwa_path", 1);
    user_path_publisher_ = node_handle_.advertise<nav_msgs::Path>("user_path", 1);
    dwa_shape_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("dwa_shape", 1);
    user_shape_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("user_shape", 1);
    cost_publisher_ = node_handle_.advertise<std_msgs::Float32MultiArray>("dwa_results", 1);

    // initialize obstacle distance parameters
    obst_dist_.base_frame_id_ = base_frame_id_;
    obst_dist_.fixed_frame_id_ = fixed_frame_id_;
    obst_dist_.local_map_radius_ = obst_range_;
    obst_dist_.max_scan_radius_ = obst_range_;
    obst_dist_.occupancy_threshold_ = occupancy_threshold_;

    // initialize markers
    dwa_footprint_marker_.type = visualization_msgs::Marker::LINE_LIST;
    dwa_footprint_marker_.color.g = 1.0;
    dwa_footprint_marker_.color.a = 1.0;
    dwa_footprint_marker_.scale.x = 0.02;

    user_footprint_marker_.type = visualization_msgs::Marker::LINE_LIST;
    user_footprint_marker_.color.r = 1.0;
    user_footprint_marker_.color.a = 1.0;
    user_footprint_marker_.scale.x = 0.02;

    dynamic_window_marker_.type = visualization_msgs::Marker::LINE_LIST;
    dynamic_window_marker_.color.b = 1.0;
    dynamic_window_marker_.color.a = 1.0;
    dynamic_window_marker_.scale.x = 0.01;
    
    // Check if all essential data are being received before starting node process.
    if (node_active_)
    {
        while (!obst_received_)
        {
            ROS_INFO("shared dwa waiting for obstacle data");
            ros::Duration(1).sleep();
            ros::spinOnce();
        }

        while (!vel_received_)
        {
            ROS_INFO("shared dwa waiting for velocity data");
            ros::Duration(1).sleep();
            ros::spinOnce();
        }
        ROS_INFO("shared dwa all data received");
    }

    ROS_INFO("Launched Shared DWA.");
}

// Destructor
DWANode::~DWANode()
{
}
//Public Member Functions

//Private Member Functions
bool DWANode::readBotParameters()
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

    if (!node_handle_.getParam("footprint", footprint_coords_))
    {
        ROS_WARN_STREAM("Parameter footprint not set for shared_dwa. Using radius collision_method.");
        collision_method_ = "radius";
    }

    if (footprint_coords_.size() < 6)
    {
        ROS_ERROR("number of footprint segments less than 3. Unable to continue.");
        return false;
    }

    w_min_ = 2 * v_min_ / base_width_;
    v_range_ = v_max_ - v_max_neg_;
    w_range_ = 2 * w_max_;
    robot_radius_ = robot_radius_ * footprint_inflation_;
    footprint_ = nav_utils::createPolygon2D(footprint_coords_, footprint_inflation_, 2);

    // Calculate the largest distance from center of rotation to corner point, for rotation collision hazard calculation.
    for (auto corner : footprint_)
    {
        float L = base_utils::euclideanDistance(corner);
        if (L > L_max_)
            L_max_ = L;
    }

    return true;
}

bool DWANode::readParameters()
{
    if (!node_handle_.getParam("node_active", node_active_))
        ROS_WARN_STREAM("Parameter node_active not set for shared_dwa. Using default setting: " << node_active_);
    if (!node_handle_.getParam("obst_type", obst_type_))
        ROS_WARN_STREAM("Parameter obst_type not set for shared_dwa. Using default setting: " << obst_type_);
    if (!node_handle_.getParam("use_adaptive", use_adaptive_))
        ROS_WARN_STREAM("Parameter use_adaptive not set for shared_dwa. Using default setting: " << use_adaptive_);
    if (!node_handle_.getParam("use_ref_trajectory", use_ref_trajectory_))
        ROS_WARN_STREAM("Parameter use_ref_trajectory not set for shared_dwa. Using default setting: " << use_ref_trajectory_);
    if (!node_handle_.getParam("use_coll_margin", use_coll_margin_))
        ROS_WARN_STREAM("Parameter use_coll_margin not set for shared_dwa. Using default setting: " << use_coll_margin_);
    if (!node_handle_.getParam("fixed_frame_id", fixed_frame_id_))
        ROS_WARN_STREAM("Parameter fixed_frame_id not set for shared_dwa. Using default setting: " << fixed_frame_id_);
    if (!node_handle_.getParam("base_frame_id", base_frame_id_))
        ROS_WARN_STREAM("Parameter base_frame_id not set for shared_dwa. Using default setting: " << base_frame_id_);
    if (!node_handle_.getParam("vel_type", vel_type_))
        ROS_WARN_STREAM("Parameter vel_type not set for shared_dwa. Using default setting: " << vel_type_);
    if (!node_handle_.getParam("collision_method", collision_method_))
        ROS_WARN_STREAM("Parameter collision_method not set for shared_dwa. Using default setting: " << collision_method_);
    if (!node_handle_.getParam("footprint_inflation", footprint_inflation_))
        ROS_WARN_STREAM("Parameter footprint_inflation not set for shared_dwa. Using default setting: " << footprint_inflation_);
    if (!node_handle_.getParam("vel_inflation", vel_inflation_))
        ROS_WARN_STREAM("Parameter vel_inflation not set for shared_dwa. Using default setting: " << vel_inflation_);
    if (!node_handle_.getParam("obst_inflation", obst_inflation_))
        ROS_WARN_STREAM("Parameter obst_inflation not set for shared_dwa. Using default setting: " << obst_inflation_);
    if (!node_handle_.getParam("pos_update_freq", pos_update_freq_))
        ROS_WARN_STREAM("Parameter pos_update_freq not set for shared_dwa. Using default setting: " << pos_update_freq_);
    if (!node_handle_.getParam("prediction_horizon", prediction_horizon_))
        ROS_WARN_STREAM("Parameter prediction_horizon not set for shared_dwa. Using default setting: " << prediction_horizon_);
    if (!node_handle_.getParam("sampling_time", sampling_time_))
        ROS_WARN_STREAM("Parameter sampling_time not set for shared_dwa. Using default setting: " << sampling_time_);
    if (!node_handle_.getParam("samples_v", samples_v_))
        ROS_WARN_STREAM("Parameter samples_v not set for shared_dwa. Using default setting: " << samples_v_);
    if (!node_handle_.getParam("samples_w", samples_w_))
        ROS_WARN_STREAM("Parameter samples_w not set for shared_dwa. Using default setting: " << samples_w_);
    if (!node_handle_.getParam("occupancy_threshold", occupancy_threshold_))
        ROS_WARN_STREAM("Parameter occupancy_threshold not set for shared_dwa. Using default setting: " << occupancy_threshold_);
    if (!node_handle_.getParam("obst_range", obst_range_))
        ROS_WARN_STREAM("Parameter obst_range not set for shared_dwa. Using default setting: " << obst_range_);

    // Load constants
    if (use_adaptive_)
    {
        ROS_INFO_STREAM("Shared DWA is using Adaptive mode, Cost function tuned to individual users based on joystick filter parameters.");
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
    if (!node_handle_.getParam("Kc", Kc_))
        ROS_WARN_STREAM("Parameter Kc not set for shared_mpc. Using default setting: " << Kc_);
    if (!node_handle_.getParam("Kref", Kref_))
        ROS_WARN_STREAM("Parameter Kref not set for shared_mpc. Using default setting: " << Kref_);

    float sum_pars = Kv_ + Kw_ + Kh_ + Ks_ + Kc_ + Kref_;
    if (sum_pars > 1.0)
    {
        Kv_ = Kv_ / sum_pars;
        Kw_ = Kw_ / sum_pars;
        Kh_ = Kh_ / sum_pars;
        Ks_ = Ks_ / sum_pars;
        Kc_ = Kc_ / sum_pars;
        Kref_ = Kref_ / sum_pars;
        if (use_adaptive_)
        {
            Kv0_ = Kv0_ / sum_pars;
            Kw0_ = Kw0_ / sum_pars;
            Kh0_ = Kh0_ / sum_pars;
            Kvr_ = Kvr_ / sum_pars;
            Kwr_ = Kwr_ / sum_pars;
            Khr_ = Khr_ / sum_pars;
        }
        ROS_WARN_STREAM("Warning: sum of cost function parameters larger than 1. Parameters have been automatically adjusted, but this should not happen. Change parameter settings in the config file.");
    }

    // When debugging, set logger_level to Debug by changing Info to Debug.
    bool debug = false;
    if (!node_handle_.getParam("debug", debug))
        ROS_WARN_STREAM("Parameter debug not set for sparse_voronoi. Using default setting: " << debug);
    if (debug)
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    else
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // calculate the sample size of the dynamic window
    size_dw_ = static_cast<float>(samples_v_ * samples_w_);

    //
    if (obst_type_ == "map")
    {
        collision_method_ = "radius";
        ROS_INFO_STREAM("Shared DWA is using cost map for obstacle avoidance. Setting 'collision_method' to 'radius' ");
    }
    else if (obst_type_ == "obstmsg")
    {
        collision_method_ = "footprint";
        ROS_INFO_STREAM("Shared DWA is using parametrized obstacle list for obstacle avoidance. Setting 'collision_method' to 'radius' ");
    }
    if (collision_method_ == "radius" && !(obst_type_ == "map" || obst_type_ == "scan"))
        obst_type_ = "scan";

    ROS_DEBUG("Parameters read.");
    return true;
}

void DWANode::perfCallback(const std_msgs::Float32MultiArray::ConstPtr &msg_perf)
{
    if (node_active_ && use_adaptive_)
    {
        float var_x = std::max(std::min(msg_perf->data[0], 1.0f), 0.0f);
        float var_y = std::max(std::min(msg_perf->data[1], 1.0f), 0.0f);
        float var_xy = pow(pow(var_x, 2) + pow(var_y, 2), 0.5);
        float cov_xy = std::max(std::min(msg_perf->data[2], 1.0f), 0.0f);
        Kv_ = Kv0_ + Kvr_ * (1 - var_x);
        Kw_ = Kw0_ + Kwr_ * (1 - var_y);
        Kh_ = Kh0_ + Khr_ * (1 - var_xy);
    }
}

void DWANode::velCallback(const geometry_msgs::Twist::ConstPtr &msg_vel)
{
    if (node_active_)
    {
        v_agent_ = msg_vel->linear.x;
        w_agent_ = msg_vel->angular.z;
        vel_received_ = true;
    }
}

void DWANode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom)
{
    if (node_active_)
    {
        v_agent_ = msg_odom->twist.twist.linear.x;
        w_agent_ = msg_odom->twist.twist.angular.z;
        vel_received_ = true;
    }
}

void DWANode::trajCallback(const nav_msgs::Path::ConstPtr &msg_traj)
{
    if (node_active_)
    {
        // Local reference trajectory which can be included in the cost function
        if (msg_traj->poses.size() != prediction_horizon_)
        {
            ROS_ERROR_STREAM("Shared DWA received reference trajectory with size " << msg_traj->poses.size() << ", while its prediction_horizon is set to " << prediction_horizon_ << "Ignoring Trajectory.");

        }
        else if (msg_traj->header.frame_id != base_frame_id_)
        {
            ROS_ERROR_STREAM("Shared DWA received reference trajectory with frame_id " << msg_traj->header.frame_id << ", while local frame base_frame_id is set to " << base_frame_id_ <<". These should be the same. Ignoring trajectory.");
        }
        else
        {
            ref_trajectory_.poses = msg_traj->poses;
            ref_trajectory_.header = msg_traj->header;
            ref_trajectory_received_ = true;            
        }
    }
}

void DWANode::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg_costmap)
{
    if (node_active_ && obst_type_ == "map")
    {
        obst_dist_.updateObsgrid(*msg_costmap);
        obst_received_ = true;
    }
}

void DWANode::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_scan)
{
    if (node_active_ && obst_type_ == "scan")
    {
        sensor_msgs::PointCloud cloud;
        try
        {
            tf_listener_.waitForTransform(msg_scan->header.frame_id, base_frame_id_,
                                          msg_scan->header.stamp + ros::Duration().fromSec(msg_scan->ranges.size() * msg_scan->time_increment),
                                          ros::Duration(1.0));
            projector_.transformLaserScanToPointCloud(base_frame_id_, *msg_scan, cloud, tf_listener_);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("Vel_filter: %s", ex.what());
            return;
        }

        point_cloud_.points.clear();
        point_cloud_.header = cloud.header;
        for (auto &point : cloud.points)
        {
            if (base_utils::euclideanDistance(point) < obst_range_)
            {
                point_cloud_.points.push_back(point);
            }
        }
        // ROS_DEBUG_STREAM("size point cloud: " << scan_cloud_.points.size());

        if (collision_method_ == "radius" || use_coll_margin_) // the obst_dist_ algorithm is a bit more expensive, so preferably not used.
            obst_dist_.updateObsgrid(point_cloud_);

        obst_received_ = true;
    }
}

void DWANode::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_cloud)
{
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud cloud;

    try
    {
        tf_listener_.waitForTransform(msg_cloud->header.frame_id, base_frame_id_,
                                      msg_cloud->header.stamp + ros::Duration().fromSec(0.01),
                                      ros::Duration(1.0));
        pcl_ros::transformPointCloud(base_frame_id_, *msg_cloud, cloud2, tf_listener_);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("Vel_filter: %s", ex.what());
        return;
    }
    sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud);
    point_cloud_.points.clear();
    point_cloud_.header = cloud.header;
    for (auto &point : cloud.points)
    {
        if (base_utils::euclideanDistance(point) < obst_range_ && point.z > height_threshold_)
        {
            point_cloud_.points.push_back(point);
        }
    }
    if (collision_method_ == "radius" || use_coll_margin_) // the obst_dist_ algorithm is a bit more expensive, so preferably not used.
        obst_dist_.updateObsgrid(point_cloud_);

    obst_received_ = true;
}

void DWANode::obstCallback(const scat_msgs::EnvObjectList::ConstPtr &msg_obst)
{
    if (node_active_ && obst_type_ == "obstmsg")
    {
        // obstacles_pts_ is a vector containing obstacles, each described by a vector of (ros) points.
        obstacles_pts_ = obst_utils::convertObjectListToPoints(*msg_obst, base_frame_id_, tf_listener_);
        obst_received_ = true;
    }
}

void DWANode::userinputCallback(const geometry_msgs::Twist::ConstPtr &msg_userinput)
{
    if (node_active_)
    {
        v_user_ = msg_userinput->linear.x;
        v_user_ = std::max(std::min(v_max_, v_user_), v_max_neg_);
        w_user_ = msg_userinput->angular.z;
        w_user_ = std::max(std::min(w_max_, w_user_), -w_max_);
        theta_user_ = atan2(w_user_, v_user_);

        userinput_received_ = true;
        // Every period, determined by min_dt, a new velocity pair will be calculated.
        // Update time measurement
        t_now_ = ros::Time::now();
        dt_ = t_now_ - t_prev_;
        if (dt_.toSec() > sampling_time_)
        {
            t_prev_ = t_now_;
            header_.frame_id = base_frame_id_;
            header_.stamp = t_now_;
            if (obst_received_)
            {
                if ((fabs(v_user_) > v_threshold_) || (fabs(w_user_) > w_threshold_))
                {
                    publishResults();
                }
                else
                    cmd_publisher_.publish(rosmsg::makeTwist(0, 0));
            }
        }
    }
}

void DWANode::timerCallback(const ros::TimerEvent &)
{
    if (node_active_)
    {
        // The robot needs to track its position in the map.
        // This timer updates the obst_dist algorithm to keep track of its global position.
        if (collision_method_ == "radius")
            ROS_INFO_STREAM(" Distance to nearest obstacle: " << obst_dist_.get_min_distance(0, 0));
        if (obst_type_ == "map" && obst_received_)
        {
            obst_dist_.timestamp = t_now_;
            obst_dist_.updatePositionMap();
        }
    }
}

void DWANode::generateDynamicWindow()
{
    v_agent_ = std::max(std::min(v_max_, v_agent_), v_max_neg_);
    w_agent_ = std::max(std::min(w_max_, w_agent_), -w_max_);

    float v_lower, v_upper, w_lower, w_upper;
    // Compute the boudaries of dynamic window
    if (v_user_ < 0)
    {
        v_lower = std::max(v_agent_ - v_acc_ * sampling_time_, v_max_neg_);
        v_upper = 0;
    }
    else
    {
        v_lower = 0;
        v_upper = std::min(v_agent_ + v_acc_ * sampling_time_, v_max_);
    }

    w_lower = std::max(w_agent_ - w_acc_ * sampling_time_, -w_max_);
    w_upper = std::min(w_agent_ + w_acc_ * sampling_time_, w_max_);

    v_dw_vector_ = base_utils::linspace(v_lower, v_upper, samples_v_);
    w_dw_vector_ = base_utils::linspace(w_lower, w_upper, samples_w_);

    ROS_DEBUG_STREAM("max v dwa: " << v_upper << " min v dwa: " << v_lower);
    ROS_DEBUG_STREAM("max w dwa: " << w_upper << " min w dwa: " << w_lower);

    //// Can be used for producing terminal output of dynamic window.
    //std::cout << "v: " << std::endl;
    //for (auto v : v_dw_vector_)
    //     std::cout << v << "  ";
    // std::cout << std::endl;
    // std::cout << "w: " << std::endl;
    // for (auto w : w_dw_vector_)
    //     std::cout << w << "  ";
    // std::cout << std::endl;
}

void DWANode::selectVelocity()
{
    dynamic_window_marker_.points.clear();
    dynamic_window_marker_.header = rosmsg::makeHeader(base_frame_id_, t_now_);
    // v_prev_ = v_dwa_;
    // w_pref_ = w_dwa_;
    dur1_ = std::chrono::duration<double>::zero(); 
    dur2_ = std::chrono::duration<double>::zero(); 
    float opt_clearance, opt_safety, opt_heading, opt_velocity, opt_smoothness, opt_ref_traj, min_cost = 1000;
    float obj_function, cost_heading, cost_velocity, cost_smoothness,
        size_path = 0, clearance = 0, cost_safety = 0, cost_ref_traj = 0;
    av_danger_ = 0, av_path_length_ = 0, frac_safe_paths_ = 0;
    float av_traj = 0;
    int nr_safe_paths = 0;
    ros::Time t1 = ros::Time::now();
    ROS_DEBUG_STREAM("v_user_: " << v_user_ << " w_user_: " << w_user_);
    ROS_DEBUG_STREAM("v_agent_: " << v_agent_ << " w_agent_: " << w_agent_);
    ROS_DEBUG_STREAM("Kv: " << Kv_ << " Kh: " << Kh_ << " Kw: " << Kw_);
    ROS_DEBUG_STREAM("Kc: " << Kc_ << " Ks: " << Ks_ << " Kref: " << Kref_);
    // Loop over all values in dynamic window and calculate objective function.
    for (float v : v_dw_vector_)
    {
        // Add additional inflation depending on velocity.
        float inflation = footprint_inflation_ + vel_inflation_ * std::max(fabs(v) - v_min_, 0.0) / (v_max_ - v_min_);
        footprint_ = nav_utils::createPolygon2D(footprint_coords_, inflation, 2);
        for (float w : w_dw_vector_)
        {
            //ROS_WARN_STREAM("v: " << v << " w: " << w);
            vel_utils_.limitVelocity(v, w);
            // Calculate clerance from the normalized obstacle distance metric stored in dist_window.
            predictTrajectory(v, w, size_path, clearance, cost_safety, cost_ref_traj);
            // dangerousness is defined as the average clearance for all velocity pairs.
            av_danger_ += (1 - clearance); // can be used to improve robustness as done in shared_dwa paper / tune max velocity
            av_path_length_ += size_path;  // can be used to tune max velocity based on environment.
            if (static_cast<int>(size_path) == path_samples_ - 1)
                nr_safe_paths += 1;
            // Calculate normalized heading = disparity to the direction indicated by user
            cost_heading = calcHeading(v, w, theta_user_);
            // Calculate normalized velocity = disparity to the velocity indicated by user
            cost_velocity = calcVelocity(v, w, v_user_, w_user_);
            // Calculate smoothness cost = disparity between these commands and previous
            cost_smoothness = calcSmoothness(v, w);
            // compute current objective value
            obj_function = objFunction(clearance, cost_heading, cost_velocity, cost_smoothness, cost_safety, cost_ref_traj);
            //// Can be used to visualize DWA evaluations
            //ROS_DEBUG_STREAM("v: " << v << " w: " << w );
            //ROS_DEBUG_STREAM("clearance: " << clearance << " heading: " << cost_heading << " velocity: " << cost_velocity );
            //ROS_DEBUG_STREAM("obj_function: " << obj_function );
            //ROS_DEBUG_STREAM("size_path: " << size_path );

            // if current obj value is larger than max obj value
            // then replace max obj value and its row and col
            // ROS_DEBUG_STREAM("v: " << v << "w: " << w);
            // ROS_DEBUG_STREAM("cost: " << obj_function  << " cost_velocity: " << cost_velocity <<" cost_heading: " << cost_heading << " clearance: " << clearance << "cost_smoothness: " << cost_smoothness);
            av_traj += cost_ref_traj;
            if (obj_function < min_cost)
            {
                min_cost = obj_function;
                // Store values for debugging
                opt_clearance = clearance;
                opt_heading = cost_heading;
                opt_velocity = cost_velocity;
                opt_smoothness = cost_smoothness;
                opt_safety = cost_safety;
                opt_ref_traj = cost_ref_traj;

                // Select optimal velocities
                v_dwa_ = v;
                w_dwa_ = w;
                // ROS_DEBUG_STREAM("heading user: " << atan2(v_user_,w_user_) << " heading dwa: " << atan2(v,w));
                // ROS_ERROR_STREAM("OPTIMAL");
            }
        }
    }
    // // Visualize optimal values for debugging
    // ROS_DEBUG_STREAM("clearance: " << opt_clearance << " heading: " << opt_heading << " velocity: " << opt_velocity);
    // ROS_DEBUG_STREAM("smoothness: " << opt_smoothness << " safety: " << opt_safety << " ref_traj: " << opt_ref_traj);
    // ROS_DEBUG_STREAM("total cost: " << min_cost);
    // ROS_DEBUG_STREAM("av traj: " << av_traj/size_dw_);
    ros::Time t2 = ros::Time::now();
    ros::Duration dt = t2 - t1;
    ROS_DEBUG_STREAM("Time to calculate objective: " << dt);
    ROS_DEBUG_STREAM("Total number of samples calculated: " << av_path_length_);
    // float clearance_threshold = 0.2;
    // if (opt_clearance < clearance_threshold)
    // {
    //     v_dw = 0;
    //     w_dw = 0;
    //     ROS_WARN_STREAM("Collision detected for optimal path. Reset to 0");
    // }

    v_dw_vector_.clear();
    w_dw_vector_.clear();
    // store danger to be used next loop (e.g. to decrease velocity)
    av_path_length_ = av_path_length_ / size_dw_;
    av_danger_ = av_danger_ / size_dw_;
    frac_safe_paths_ = static_cast<float>(nr_safe_paths) / size_dw_;
    // // all costs and metrics can be stored in one array, useful for plotting and debugging.
    dwa_cost_.data.clear();
    dwa_cost_.data.push_back(min_cost);
    dwa_cost_.data.push_back(opt_clearance);
    dwa_cost_.data.push_back(opt_heading);
    dwa_cost_.data.push_back(opt_velocity);
    dwa_cost_.data.push_back(opt_smoothness);
    dwa_cost_.data.push_back(opt_safety);
    dwa_cost_.data.push_back(av_path_length_);
    dwa_cost_.data.push_back(av_danger_);
    dwa_cost_.data.push_back(frac_safe_paths_);
}

void DWANode::publishResults()
{
    if (vel_received_ && obst_received_ && userinput_received_)
    {
        generateDynamicWindow();
        selectVelocity();

        ROS_DEBUG_STREAM("DWA Linear Velocity: " << v_dwa_ << " Angular Velocity: " << w_dwa_);
        ROS_DEBUG_STREAM("total trajectory calculation time [ms] " << dur1_.count()*1000);
        ROS_DEBUG_STREAM("total collision checking time [ms] " << dur2_.count()*1000);

        dwa_path_ = diff_drive::makePath(v_dwa_, w_dwa_, prediction_horizon_, sampling_time_,
                                         rosmsg::makeHeader(base_frame_id_, t_now_));
        float inflation = footprint_inflation_ + (fabs(v_dwa_) - v_min_) / (v_max_ - v_min_) * vel_inflation_;
        footprint_ = nav_utils::createPolygon2D(footprint_coords_, inflation, 2);
        dwa_footprint_marker_.header = rosmsg::makeHeader(base_frame_id_, t_now_);
        dwa_footprint_marker_.points = rosmsg::makeMarkerPoints(nav_utils::moveFootprint(footprint_,
                                                                                         dwa_path_.poses.back().pose.position.x,
                                                                                         dwa_path_.poses.back().pose.position.y,
                                                                                         tf::getYaw(dwa_path_.poses.back().pose.orientation)));

        user_path_ = diff_drive::makePath(v_user_, w_user_, prediction_horizon_, sampling_time_,
                                          rosmsg::makeHeader(base_frame_id_, t_now_));
        inflation = footprint_inflation_ + (fabs(v_user_) - v_min_) / (v_max_ - v_min_) * vel_inflation_;
        footprint_ = nav_utils::createPolygon2D(footprint_coords_, inflation, 2);
        user_footprint_marker_.header = rosmsg::makeHeader(base_frame_id_, t_now_);
        user_footprint_marker_.points = rosmsg::makeMarkerPoints(nav_utils::moveFootprint(footprint_,
                                                                                          user_path_.poses.back().pose.position.x,
                                                                                          user_path_.poses.back().pose.position.y,
                                                                                          tf::getYaw(user_path_.poses.back().pose.orientation)));

        dwa_path_publisher_.publish(dwa_path_);
        user_path_publisher_.publish(user_path_);
        dwa_shape_publisher_.publish(dwa_footprint_marker_);
        user_shape_publisher_.publish(user_footprint_marker_);
        dw_marker_publisher_.publish(dynamic_window_marker_);
        cost_publisher_.publish(dwa_cost_);

        // difference between input and output, for haptic feedback
        if (pub_shared_vel_)
        {
            scat_msgs::SharedVelocity shared_vel;
            shared_vel.cmd_vel = rosmsg::makeTwist(v_dwa_, w_dwa_);
            shared_vel.user_vel = rosmsg::makeTwist(v_user_, w_user_);
            diff_cmd_publisher_.publish(shared_vel);
        }

        cmd_publisher_.publish(rosmsg::makeTwist(v_dwa_, w_dwa_));
    }
    else
    {
        ROS_WARN("Data not yet received. Unable to generate Dynamic Window.");
        ROS_INFO_STREAM("velocity: " << vel_received_ << " obstacle data: " << obst_received_ << " userinput: " << userinput_received_);
    }
}

// Extrapolates a trajectory based on velocity commands v_dw and w_dw.
// it returns a number of metrics through the references:
// size_path: number of steps until end of trajectory or obstacle collision
// clearance: metric for how far away v_dw and w_dw are from an inevitable collision (detrmined by v_inev, w_inev)
// safety: average distance to obstacles along the path
// cost_ref_traj: average distance to some reference trajectory along the path.
void DWANode::predictTrajectory(float v_dw, float w_dw, float &size_path, float &clearance, float &cost_safety, float &cost_ref_traj)
{
    cost_ref_traj = 0.0;
    cost_safety = 0.0;
    // Obstacle distance
    float dist_collision = 0.0;
    float angle_collision = 0.0;
    float av_dist = 0.0;
    float obst_inflation = obst_inflation_ * (1 + vel_inflation_ * std::max(fabs(v_dw) - v_min_, 0.0) / (v_max_ - v_min_));

    // variables for predicted path coordinates
    geometry_msgs::Pose2D agent_pose = rosmsg::makePose2D(0, 0, 0);
    // initialize the angular distance and linear distance
    angle_collision = M_PI;
    dist_collision = 20;
    // Alternative prediction horizon
    float v_ref = std::max(fabs(v_agent_), fabs(v_dw));
    float w_ref = std::max(fabs(w_agent_), fabs(w_dw));

    if (use_ref_trajectory_ && ref_trajectory_received_) // if using ref trajectory, predicted trajectory needs to have the exact same path length.
        path_samples_ = prediction_horizon_;
    else
    {
        float dec_time = std::max(v_ref / v_dec_, w_ref / w_dec_) +
                         +sqrt(2 * obst_inflation / std::min(v_dec_, w_dec_ * L_max_)) + sampling_time_;
        path_samples_ = static_cast<int>(ceil(dec_time / sampling_time_));
    }

    // find the intersection of trajectory curvature and lidar range data
    for (int i = 0; i < path_samples_; i++)
    {
        size_path = static_cast<float>(i);
        t0_ = std::chrono::system_clock::now();

        // Step agent along trajectory, given current position and velocities.
        diff_drive::takeStep(agent_pose, v_dw, w_dw, sampling_time_);

        t1_ = std::chrono::system_clock::now();

        if (use_ref_trajectory_ && ref_trajectory_received_)
        {
            cost_ref_traj += base_utils::euclideanDistance(ref_trajectory_.poses[i].pose.position, agent_pose);
        }

        if (collision_method_ == "radius")
        {
            float obst_dist = obst_dist_.get_min_distance(agent_pose.x, agent_pose.y);
            av_dist += obst_dist;
            if (obst_dist <= obst_inflation)
            {
                // select current angle as the angular distance and calculate corresponding linear distance
                angle_collision = w_dw * size_path * sampling_time_;
                dist_collision = v_dw * size_path * sampling_time_;
                break;
            }
        }
        else if (collision_method_ == "footprint")
        {
            if (obst_type_ == "scan" || obst_type_ == "cloud")
            {
                if (use_coll_margin_) // this calculation is a bit expensive. Only do when we're using the collision cost.
                {
                    float obst_dist = obst_dist_.get_min_distance(agent_pose.x, agent_pose.y);
                    av_dist += obst_dist;
                }
                if (nav_utils::checkFootprintForCollision(footprint_, point_cloud_, agent_pose.x, agent_pose.y, agent_pose.theta))
                {
                    // select current angle as the angular distance and calculate corresponding linear distance
                    angle_collision = w_dw * size_path * sampling_time_;
                    dist_collision = v_dw * size_path * sampling_time_;
                    // ROS_DEBUG_STREAM("collision at dist " << dist_collision << " angle " << angle_collision);
                    break;
                }
            }
            else if (obst_type_ == "obstmsg")
            {
                if (nav_utils::checkFootprintForCollision(footprint_, obstacles_pts_, agent_pose.x, agent_pose.y, agent_pose.theta))
                {
                    // select current angle as the angular distance and calculate corresponding linear distance
                    angle_collision = w_dw * size_path * sampling_time_;
                    dist_collision = v_dw * size_path * sampling_time_;
                    // ROS_DEBUG_STREAM("collision at dist " << dist_collision << " angle " << angle_collision);
                    break;
                }
            }
            else
                ROS_ERROR("obst_type parameter setting not recognized in Shared DWA. ");
        }
        t2_ = std::chrono::system_clock::now();
        dur1_ += t1_-t0_;
        dur2_ += t2_-t1_;
        //// troubleshooting for nan path values
        // if (isnan(agent_pose.x) || (fabs(agent_pose.x) < 1e-1 && fabs(v_dw) > v_threshold_ && i > 0))
        // ROS_ERROR_STREAM("Wrong Value for X: " << agent_pose.x << " at path sample nr:" << i << " for sampled velocity V = " << v_dw << " and W = " << w_dw) ;
        // if (isnan(agent_pose.y) || (fabs(agent_pose.y) < 1e-1 && fabs(v_dw) > v_threshold_ && i > 0))
        // ROS_ERROR_STREAM("Wrong Value for Y: " << agent_pose.y << " at path sample nr: " << i << " for sampled velocity V = " << v_dw << " and W = " << w_dw << " x = " << agent_pose.x << " theta = " << agent_pose.theta);
        // if (isnan(agent_pose.theta) || agent_pose.theta < 1e-2)
        // ROS_ERROR_STREAM("Wrong Value for theta: " << agent_pose.theta << " at path sample nr: " << i << " for sampled velocity V = " << v_dw << " and W = " << w_dw);
    }
    // store end points for visualization
    dynamic_window_marker_.points.push_back(rosmsg::makePoint(0.0, 0.0));
    dynamic_window_marker_.points.push_back(rosmsg::makePoint(agent_pose.x, agent_pose.y));
    // // Determine the Shared DWA definition of collision distance and store in distance matrix
    // float v_inev = sqrt(2 * v_dec_ * dist_collision);     // Maximum speed such that agent can still do emergency brake
    // float w_inev = sqrt(2 * w_dec_ * angle_collision);    // Maximum speed such that agent can still do emergency brake

    //// The returned values are all calculated below: clearance, safety, and cost_ref_traj.
    // metric for distance velocity commands to inevitable collision commands
    if (dist_collision <= obst_inflation || angle_collision <= asin(obst_inflation / L_max_)) // collision detected
        clearance = 0;
    else
        clearance = size_path / static_cast<float>(path_samples_);

    // // metric for distance velocity commands to inevitable collision commands, seems to work less well.
    // if (dist_collision <= obst_inflation || angle_collision <= asin(obst_inflation/L_max_)) // collision detected
    //     clearance = 0;
    // else if (v_dw >= v_inev || w_dw >= w_inev) // collision detected
    //     clearance = 0;
    // // else if (fabs(v_dw) < v_threshold_) // zero velocity
    // //     clearance = 1;
    // else
    //     // clearance = pow(std::min((v_inev - fabs(v_dw)) / (v_inev), (w_inev - fabs(w_dw)) / (w_inev)),2); // /(v_inev - v_min)
    //     clearance = std::min( (v_inev - fabs(v_dw))/v_inev, (w_inev - fabs(w_dw))/w_inev );

    // metric for average safety along path, = constant x average obstacle distance
    if (use_coll_margin_)
        cost_safety = Kc_ * size_path / av_dist;

    // metric for average distance to some reference trajectory
    cost_ref_traj = Kref_ * cost_ref_traj / (1 + size_path);
}

float DWANode::calcVelocity(float v_dw, float w_dw, float v_goal, float w_goal)
{
    // float cost_velocity = pow((v_dw - v_goal) / v_max_, 2);
    float cost_velocity = Kv_ * fabs(v_dw - v_goal) / v_max_ + Kw_ * fabs(w_dw - w_goal) / w_max_;
    // float cost_velocity = Kv_ * pow((v_dw - v_goal) / v_max_, 2) + Kw_ * pow((w_dw - w_goal) / w_max_, 2);
    // float cost_velocity = (1-av_danger_) * fabs(v_dw - v_goal)/(v_max_ - v_max_neg_) + av_danger_ * fabs(v_dw)/(v_max_ - v_max_neg_) ;
    return cost_velocity;
}

float DWANode::calcHeading(float v_dw, float w_dw, float theta_goal)
{
    // float cost_heading = Kh_ * pow((atan2(v_dw, w_dw) - theta_goal) / (2*M_PI), 2);
    float cost_heading = Kh_ * fabs(atan2(w_dw, v_dw) - theta_goal) / (2 * M_PI);
    return cost_heading;
}

float DWANode::calcSmoothness(float v_dw, float w_dw)
{
    // Using the velocity commands stored in the previous iteration to calculate the cost for smoothness
    float cost_smoothness = Ks_ * (pow((v_agent_ - v_dw) / (2 * v_max_), 2) + pow((w_agent_ - w_dw) / (2 * w_max_), 2));
    // float cost_smoothness = Ks_ * (pow((v_pref_ - v_dw) / (2 * v_max_), 2) + pow((w_pref_ - w_dw) / (2 * w_max_), 2));
    // float cost_smoothness = Ks_ * (fabs(cmd_dwa_.linear.x-v_dw) / (2*v_max_) + fabs(cmd_dwa_.linear.x-w_dw)/(2*w_max_));
    return cost_smoothness;
}

float DWANode::objFunction(float clearance, float cost_heading, float cost_velocity, float cost_smoothness, float cost_safety, float cost_ref_traj)
{
    // clearance = 1 means safe environment, therefore cost function = heading + velocity + smoothness
    // clearance = 0 means collision, therfore cost function = 1
    // float obj_function = cost_heading + cost_velocity;
    // float obj_function = (1 - clearance) + clearance * (cost_heading + cost_velocity);
    if (cost_heading + cost_velocity + cost_smoothness + cost_safety + cost_ref_traj > 1)
    {
        ROS_ERROR_STREAM("Right half part of shared dwa cost function larger than 1: " << cost_heading + cost_velocity + cost_smoothness + cost_safety);
        ROS_WARN_STREAM(" This should never happen! Indicates wrongly tuned cost parameters");
        ROS_WARN_STREAM("cost velocity: " << cost_velocity << " cost heading: " << cost_heading << " cost_smoothness: " << cost_smoothness << " cost safety: " << cost_safety << " cost ref traj: " << cost_ref_traj);
    }

    float obj_function = (1 - clearance) + clearance * (cost_heading + cost_velocity + cost_smoothness + cost_safety + cost_ref_traj);
    return obj_function;
}

bool DWANode::serviceCallback(std_srvs::SetBoolRequest &request,
                              std_srvs::SetBoolResponse &response)
{
    // This service can be used to activate / deactivate the node.
    node_active_ = request.data;
    response.success = true;
    ROS_WARN_STREAM("Shared DWA active: " << node_active_);
    return true;
}

} // namespace shared_dwa
