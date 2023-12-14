#include "shared_dwa/shared_DWA_node.h"
// ROS
#include <ros/ros.h>
// Messages
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
// C++
#include <math.h>
#include <chrono>
#include <future>
#include <thread>
#include <limits>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace shared_DWA
{
    // Constructor
    shared_DWANode::shared_DWANode(ros::NodeHandle &node_handle)
        : node_handle_(node_handle)
    {
        if (!readParameters())
        {
            ROS_ERROR_STREAM("Could not load parameters.");
            ros::requestShutdown();
        }
        dwa_spinner_ = new ros::AsyncSpinner(0);

        // Subscribers & Publishers
        scan_subscriber_ = node_handle_.subscribe("/scan", 1, &shared_DWANode::scanCallback, this);
        odom_subscriber_ = node_handle_.subscribe("/odom", 1, &shared_DWANode::odomCallback, this);
        cmd_subscriber_ = node_handle_.subscribe("/joystick_calib", 1, &shared_DWANode::cmdCallback, this);
        if (guidance_mode == "goal")
        {
            goal_subscriber_ = node_handle_.subscribe("/goal_distribution", 1, &shared_DWANode::goalCallback, this);
            global_subscriber_ = node_handle_.subscribe("/belief_update/global_goal", 1, &shared_DWANode::globalCallback, this);
        }

        vel_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/shared_dwa/cmd_vel", 1);
        dwa_vel_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/shared_dwa/pure_dwa_vel", 1);
        // goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/shared_dwa/goal", 1);
        cancel_publisher_ = node_handle_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
        algo_timer_ = node_handle_.createTimer(ros::Duration(algorithm_interval), &shared_DWANode::algotimerCallback, this);
        pub_timer_ = node_handle_.createTimer(ros::Duration(publish_interval), &shared_DWANode::pubtimerCallback, this);

        //Visualization publishers
        cddt_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/dwa_cone", 1);
        line_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/dwa_final_output", 1);
        udwa_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/dwa_follow_user", 1);
        ucmd_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/dwa_user_cmd", 1);
        rdwa_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/dwa_follow_path", 1);
        // wypt_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/waypoints", 1);
        text_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("/visualization/prob_txt", 1);
        // list_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("/visualization/line_namelist", 1);
        weight_publisher_ = node_handle_.advertise<std_msgs::Float32>("/shared_dwa/user_weight", 1);
        footprint_publisher_ = node_handle_.advertise<geometry_msgs::PolygonStamped>("/shared_dwa/footprint", 1);

        collision_marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/collision_warning", 1);

        // // initialize the final published twist
        // dwa_twist.linear.x = 0;
        // dwa_twist.linear.y = 0;
        // dwa_twist.linear.z = 0;
        // dwa_twist.angular.x = 0;
        // dwa_twist.angular.y = 0;
        // dwa_twist.angular.z = 0;
        old_dwa_twist.linear.x = 0;
        old_dwa_twist.angular.z = 0;
        
        // // initialize dwa_goal
        // dwa_goal.header.frame_id = base_frame_id;
        // dwa_goal.pose.position.x = 0;
        // dwa_goal.pose.position.y = 0;
        // dwa_goal.pose.position.z = 0;
        // dwa_goal.pose.orientation.x = 0;
        // dwa_goal.pose.orientation.y = 0;
        // dwa_goal.pose.orientation.z = 0;
        // dwa_goal.pose.orientation.w = 1;

        // visualization markers
        candidate_samples.header.frame_id = base_frame_id;
        candidate_samples.ns = "visualization";
        candidate_samples.action = visualization_msgs::Marker::ADD;
        candidate_samples.pose.orientation.w = 1.0;
        candidate_samples.id = 0;
        candidate_samples.type = visualization_msgs::Marker::SPHERE_LIST;
        candidate_samples.scale.x = 0.05;
        candidate_samples.scale.y = 0.05;
        candidate_samples.scale.z = 0.05;
        candidate_samples.color.r = 1.0;
        candidate_samples.color.g = 0.75;
        candidate_samples.color.b = 0.79;
        candidate_samples.color.a = 0.3;

        final_line.header.frame_id = base_frame_id;
        final_line.ns = "visualization";
        final_line.action = visualization_msgs::Marker::ADD;
        final_line.pose.orientation.w = 1.0;
        final_line.id = 1;
        final_line.type = visualization_msgs::Marker::LINE_STRIP;
        final_line.scale.x = 0.07;
        final_line.color.r = 0.5;
        final_line.color.g = 0.25;
        final_line.color.a = 1.0;

        udwa_line.header.frame_id = base_frame_id;
        udwa_line.ns = "visualization";
        udwa_line.action = visualization_msgs::Marker::ADD;
        udwa_line.pose.orientation.w = 1.0;
        udwa_line.id = 2;
        udwa_line.type = visualization_msgs::Marker::LINE_STRIP;
        udwa_line.scale.x = 0.05;
        udwa_line.color.r = 1.0;
        udwa_line.color.a = 1.0;

        cmd_line.header.frame_id = base_frame_id;
        cmd_line.ns = "visualization";
        cmd_line.action = visualization_msgs::Marker::ADD;
        cmd_line.pose.orientation.w = 1.0;
        cmd_line.id = 3;
        cmd_line.type = visualization_msgs::Marker::LINE_STRIP;
        cmd_line.scale.x = 0.05;
        cmd_line.color.b = 1.0;
        cmd_line.color.a = 1.0;

        robot_line.header.frame_id = base_frame_id;
        robot_line.ns = "visualization";
        robot_line.action = visualization_msgs::Marker::ADD;
        robot_line.pose.orientation.w = 1.0;
        robot_line.id = 4;
        robot_line.type = visualization_msgs::Marker::LINE_STRIP;
        robot_line.scale.x = 0.05;
        robot_line.color.g = 1.0;
        robot_line.color.a = 1.0;

        // waypoint_viz.header.frame_id = base_frame_id;
        // waypoint_viz.ns = "visualization";
        // waypoint_viz.action = visualization_msgs::Marker::ADD;
        // waypoint_viz.pose.orientation.w = 1.0;
        // waypoint_viz.id = 5;
        // waypoint_viz.type = visualization_msgs::Marker::SPHERE_LIST;
        // waypoint_viz.scale.x = 0.2;
        // waypoint_viz.scale.y = 0.2;
        // waypoint_viz.scale.z = 0.2;
        // waypoint_viz.color.r = 1.0;
        // waypoint_viz.color.g = 1.0;
        // waypoint_viz.color.a = 1.0;

        waypoint_txt.header.frame_id = base_frame_id;
        waypoint_txt.ns = "visualization";
        waypoint_txt.action = visualization_msgs::Marker::ADD;
        waypoint_txt.pose.orientation.w = 1.0;
        waypoint_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        waypoint_txt.scale.z = 0.5;
        waypoint_txt.color.r = 0.0;
        waypoint_txt.color.g = 0.0;
        waypoint_txt.color.b = 0.0;
        waypoint_txt.color.a = 1.0;

        collision_marker_.header.frame_id = base_frame_id;
        collision_marker_.ns = "visualization";
        collision_marker_.id = 1.0;
        collision_marker_.type = visualization_msgs::Marker::CUBE;
        collision_marker_.action = visualization_msgs::Marker::ADD;
        collision_marker_.pose.orientation.z = 0.25;
        collision_marker_.scale.x = 0.8;
        collision_marker_.scale.y = 0.6;
        collision_marker_.scale.z = 0.5;
        collision_marker_.color.a = 0.5;
        collision_marker_.color.r = 1.0;
        collision_marker_.lifetime = ros::Duration(2 * algorithm_interval);

        // prev_time = std::chrono::system_clock::now();
        // current_time = std::chrono::system_clock::now();
        // prev_time = ros::Time::now();
        // signal(SIGINT, dwaSigintHandler);
        // ros::spin();
        // std::cout << "test" << std::endl;
        dwa_spinner_->start();
        ros::Duration(2.0).sleep();
        ros::waitForShutdown();
    }
    // Destructor
    shared_DWANode::~shared_DWANode()
    {
    }
    //Public Member Functions

    //Private Member Functions
    bool shared_DWANode::readParameters()
    {
        ROS_INFO_STREAM("Loading parameters.....");
        if (!node_handle_.getParam("algorithm_interval", algorithm_interval))
            ROS_WARN_STREAM("Parameter algorithm_interval not set. Using default setting: " << algorithm_interval);
        if (!node_handle_.getParam("publish_interval", publish_interval))
            ROS_WARN_STREAM("Parameter publish_interval not set. Using default setting: " << publish_interval);
        if (!node_handle_.getParam("sample_distance", sample_distance))
            ROS_WARN_STREAM("Parameter sample_distance not set. Using default setting: " << sample_distance);
        if (!node_handle_.getParam("sample_time", sample_time))
            ROS_WARN_STREAM("Parameter sample_time not set. Using default setting: " << sample_time);
        if (!node_handle_.getParam("v_sample", v_sample))
            ROS_WARN_STREAM("Parameter v_sample not set. Using default setting: " << v_sample);
        if (!node_handle_.getParam("w_sample", w_sample))
            ROS_WARN_STREAM("Parameter w_sample not set. Using default setting: " << w_sample);
        if (!node_handle_.getParam("v_acclrt", v_acclrt))
            ROS_WARN_STREAM("Parameter v_acclrt not set. Using default setting: " << v_acclrt);
        if (!node_handle_.getParam("w_acclrt", w_acclrt))
            ROS_WARN_STREAM("Parameter w_acclrt not set. Using default setting: " << w_acclrt);
        if (!node_handle_.getParam("v_max_robot", v_max_robot))
            ROS_WARN_STREAM("Parameter v_max_robot not set. Using default setting: " << v_max_robot);
        if (!node_handle_.getParam("w_max_robot", w_max_robot))
            ROS_WARN_STREAM("Parameter w_max_robot not set. Using default setting: " << w_max_robot);
        if (!node_handle_.getParam("footprint_mode", footprint_mode))
            ROS_WARN_STREAM("Parameter footprint_mode not set. Using default setting: " << footprint_mode);
        if (!node_handle_.getParam("fixed_frame_id", fixed_frame_id))
            ROS_WARN_STREAM("Parameter fixed_frame_id not set. Using default setting: " << fixed_frame_id);
        if (!node_handle_.getParam("base_frame_id", base_frame_id))
            ROS_WARN_STREAM("Parameter base_frame_id not set. Using default setting: " << base_frame_id);
        if (!node_handle_.getParam("weight_heading", weight_heading))
            ROS_WARN_STREAM("Parameter weight_heading not set. Using default setting: " << weight_heading);
        if (!node_handle_.getParam("weight_velocity", weight_velocity))
            ROS_WARN_STREAM("Parameter weight_velocity not set. Using default setting: " << weight_velocity);
        if (!node_handle_.getParam("weight_goal", weight_goal))
            ROS_WARN_STREAM("Parameter weight_goal not set. Using default setting: " << weight_goal);
        if (!node_handle_.getParam("weight_cmd", weight_cmd))
            ROS_WARN_STREAM("Parameter weight_cmd not set. Using default setting: " << weight_cmd);
        if (!node_handle_.getParam("weight_cmd_lb", weight_cmd_lb))
            ROS_WARN_STREAM("Parameter weight_cmd_lb not set. Using default setting: " << weight_cmd_lb);
        if (!node_handle_.getParam("enable_dynamic_weight", enable_dynamic_weight))
            ROS_WARN_STREAM("Parameter enable_dynamic_weight not set. Using default setting: " << enable_dynamic_weight);
        if (!node_handle_.getParam("guidance_mode", guidance_mode))
            ROS_WARN_STREAM("Parameter guidance_mode not set. Using default setting: " << guidance_mode);
        if (!node_handle_.getParam("weight_distance", weight_distance))
            ROS_WARN_STREAM("Parameter weight_distance not set. Using default setting: " << weight_distance);
        if (!node_handle_.getParam("tf_buffer_timeout", tf_buffer_timeout))
            ROS_WARN_STREAM("Parameter tf_buffer_timeout not set. Using default setting: " << tf_buffer_timeout);
        if (!node_handle_.getParam("enable_visualization", enable_visualization))
            ROS_WARN_STREAM("Parameter enable_visualization not set. Using default setting: " << enable_visualization);
        if (!node_handle_.getParam("dynamic_window_time_interval", dynamic_window_time_interval))
            ROS_WARN_STREAM("Parameter dynamic_window_time_interval not set. Using default setting: " << dynamic_window_time_interval);
        if (!node_handle_.getParam("gamma_cmd", gamma_cmd))
            ROS_WARN_STREAM("Parameter gamma_cmd not set. Using default setting: " << gamma_cmd);
        if (!node_handle_.getParam("gamma_goal", gamma_goal))
            ROS_WARN_STREAM("Parameter gamma_goal not set. Using default setting: " << gamma_goal);
        if (!node_handle_.getParam("min_clearance_threshold", min_clearance_threshold))
            ROS_WARN_STREAM("Parameter min_clearance_threshold not set. Using default setting: " << min_clearance_threshold);
        if (footprint_mode == "radius")
        {
            if (!node_handle_.getParam("r_collision", r_collision))
            {
                ROS_WARN_STREAM("Parameter r_collision not set. Using default setting: " << r_collision);
            }
            geometry_msgs::Point32 circle_point;
            for (int i = 0; i < 18; ++i)
            {
                circle_point.x = r_collision * cos(i * M_PI / 9);
                circle_point.y = r_collision * sin(i * M_PI / 9);
                footprint_polygon.polygon.points.push_back(circle_point);
                footprint_polygon.header.frame_id = base_frame_id;
            }
        }
        else if (footprint_mode == "rectangle")
        {
            rectangle_point.clear();
            std::vector<float> temp_list;
            geometry_msgs::Point32 temp_point;
            if (node_handle_.getParam("rectangle_point", temp_list))
            {
                for (int i = 0; i < temp_list.size(); i = i + 2)
                {
                    temp_point.x = temp_list[i];
                    temp_point.y = temp_list[i + 1];
                    rectangle_point.push_back(temp_point);
                    footprint_polygon.polygon.points.push_back(temp_point);
                    footprint_polygon.header.frame_id = base_frame_id;
                }
            }
            else
            {
                ROS_WARN_STREAM("Rectangle footprint not set, use default radius instead. Radius: " << r_collision);
                footprint_mode = "radius";
                geometry_msgs::Point32 circle_point;
                for (int i = 0; i < 18; ++i)
                {
                    circle_point.x = r_collision * cos(i * M_PI / 9);
                    circle_point.y = r_collision * sin(i * M_PI / 9);
                    footprint_polygon.polygon.points.push_back(circle_point);
                    footprint_polygon.header.frame_id = base_frame_id;
                }
            }
        }
        else
        {
            ROS_ERROR_STREAM("The selected footprint_mode is not supported, please refer to the options in config file!");
            return false;
        }

        if (guidance_mode == "disable")
        {
            ROS_INFO_STREAM("Guidance mode disabled.");
        }
        else if (guidance_mode == "goal")
        {
            ROS_INFO_STREAM("Guidance mode selected to \"goal\".");
        }
        else
        {
            ROS_ERROR_STREAM("The selected guidance_mode is not supported, please refer to the options in config file!");
            return false;
        }

        ROS_INFO_STREAM("Complete loading parameters.");
        return true;
    }

    // void shared_DWANode::dwaSigintHandler(int dwa_sig)
    // {
    //     dwa_twist.linear.x = 0;
    //     dwa_twist.angular.z = 0;
    //     vel_publisher_.publish(dwa_twist);
    //     dwa_spinner->stop();
    //     ros::shutdown();
    // }

    void shared_DWANode::scanCallback(const sensor_msgs::LaserScan &msg_scan)
    {
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);

        //Get static transform from lidar to base_link in case they are not in the same frame
        if (lidar2baseTransform.header.frame_id != base_frame_id && msg_scan.header.frame_id != base_frame_id)
        {
            ROS_INFO("LIDAR is not in base link frame and transform has not been found yet, finding transform");
            try
            {
                lidar2baseTransform = tf_buffer.lookupTransform(base_frame_id, msg_scan.header.frame_id, ros::Time(0), ros::Duration(tf_buffer_timeout));
                ROS_INFO("Transform found, all future scans received by shared_dwa will be transformed before being used for collision checking");
            }
            catch (tf2::TransformException &Exception)
            {
                ROS_ERROR("LIDAR tranform could not be found, shared_dwa may be incorrect");
                ROS_ERROR_STREAM(Exception.what());
            }
        }

        lidar_points.clear();
        lidar_points.reserve(msg_scan.ranges.size());
        for (int k = 0; k < msg_scan.ranges.size(); ++k)
        {
            auto calLidarStart = std::chrono::system_clock::now();
            geometry_msgs::Point temp_point;
            temp_point.x = msg_scan.ranges[k] * cos(msg_scan.angle_min + k * msg_scan.angle_increment);
            temp_point.y = msg_scan.ranges[k] * sin(msg_scan.angle_min + k * msg_scan.angle_increment);

            //If transform header is not empty
            auto transformStart = std::chrono::system_clock::now();
            if (!lidar2baseTransform.header.frame_id.empty())
                tf2::doTransform<geometry_msgs::Point>(temp_point, temp_point, lidar2baseTransform);
            lidar_points.emplace_back(std::move(temp_point));
        }
    }

    void shared_DWANode::odomCallback(const nav_msgs::Odometry &msg_odom)
    {
        v_agent = round(msg_odom.twist.twist.linear.x * 100) / 100;
        w_agent = round(msg_odom.twist.twist.angular.z * 100) / 100;
    }

    void shared_DWANode::cmdCallback(const geometry_msgs::Point &msg_cmd)
    {
        v_cmd = round(msg_cmd.x * 100) / 100;
        w_cmd = round(msg_cmd.y * 100) / 100;

        // v_cmd = (fabs(v_cmd) >= 0.05) ? v_cmd : 0;
        // w_cmd = (fabs(w_cmd) >= 0.05) ? w_cmd : 0;

        candidate_samples.header.stamp = ros::Time::now();
        cmd_receive = true;
    }

    void shared_DWANode::goalCallback(const geometry_msgs::PoseArray &msg_goal)
    {
        goal_data.poses.clear();
        goal_data = msg_goal;
        goal_receive = true;
    }

    void shared_DWANode::globalCallback(const geometry_msgs::Pose &msg_global)
    {
        global_goal = msg_global;
        global_receive = true;
    }

    void shared_DWANode::algotimerCallback(const ros::TimerEvent &)
    {
        if(cmd_receive)
        {
            if (guidance_mode == "goal")
            {
                //Goal receive is always true after first time
                if (goal_receive)
                {
                    goal_found = shared_DWANode::generateGoal();
                }
            }

            //If no joystick input
            if (v_cmd == 0 && w_cmd == 0)
            {
                dwa_twist.linear.x = 0;
                dwa_twist.angular.z = 0;
                pure_dwa_twist.linear.x = 0;
                pure_dwa_twist.angular.z = 0;
            }

            else
            {
                v_max = std::min(fabs(v_cmd), fabs(v_max_robot));
                if (v_cmd == 0.0)
                    w_max = std::min(fabs(w_cmd), fabs(w_max_robot));

                else
                    w_max = w_max_robot;

                if(!shared_DWANode::dynamicWindow())
                {
                    ROS_ERROR_STREAM("Error! Dynamic window is not generated.");
                    return;
                }
                else
                {
                    shared_DWANode::checkDestination();
                    shared_DWANode::selectVelocity();
                }
            }

            shared_DWANode::publishResults();
            pub_dwa_twist = dwa_twist;
        }
    }

    void shared_DWANode::pubtimerCallback(const ros::TimerEvent &)
    {
        geometry_msgs::Twist temp_twist = pub_dwa_twist;
        if (fabs(pub_dwa_twist.linear.x - old_dwa_twist.linear.x) > publish_interval * v_acclrt)
        {
            if (pub_dwa_twist.linear.x  > old_dwa_twist.linear.x)
            {
                temp_twist.linear.x = old_dwa_twist.linear.x + publish_interval * v_acclrt;
            }
            else
            {
                temp_twist.linear.x = old_dwa_twist.linear.x - publish_interval * v_acclrt;
            }
        }
        
        if (fabs(pub_dwa_twist.angular.z - old_dwa_twist.angular.z) > publish_interval * w_acclrt)
        {
            if (pub_dwa_twist.angular.z  > old_dwa_twist.angular.z)
            {
                temp_twist.angular.z = old_dwa_twist.angular.z + publish_interval * w_acclrt;
            }
            else
            {
                temp_twist.angular.z = old_dwa_twist.angular.z - publish_interval * w_acclrt;
            }
        }
        temp_twist.linear.x = round(temp_twist.linear.x * 1000) / 1000;
        temp_twist.angular.z = round(temp_twist.angular.z * 1000) / 1000;
        vel_publisher_.publish(temp_twist);
        old_dwa_twist = temp_twist;
        // current_time = ros::Time::now();
        // time_duration = current_time - prev_time;
        // std::cout << "time interval = " << time_duration.toSec() << std::endl;
        // std::cout << "linear v = " << temp_twist.linear.x << std::endl;
        // prev_time = current_time;
    }

    void shared_DWANode::publishResults()
    {
        dwa_vel_publisher_.publish(pure_dwa_twist);
        weight_publisher_.publish(final_cmd_weight);
        if (enable_visualization)
        {
            cddt_publisher_.publish(candidate_samples);
            line_publisher_.publish(final_line);
            udwa_publisher_.publish(udwa_line);
            ucmd_publisher_.publish(cmd_line);
            rdwa_publisher_.publish(robot_line);
            // wypt_publisher_.publish(waypoint_viz);
            text_publisher_.publish(waypoint_prob);
            // list_publisher_.publish(line_namelist);
            footprint_publisher_.publish(footprint_polygon);
        }
        // if(dwa_goal.header.frame_id != "")
        //     goal_publisher_.publish(dwa_goal);

        candidate_samples.points.clear();
        final_line.points.clear();
        udwa_line.points.clear();
        cmd_line.points.clear();
        robot_line.points.clear();
        // waypoint_viz.points.clear();
        waypoint_prob.markers.clear();
        // line_namelist.markers.clear();
        dynamic_window.release();
        path_cost_window.release();
        dist_cost_window.release();
        angle_cost_window.release();
        clearance_window.release();
        heading_window.release();
        velocity_window.release();
        cmd_receive = false;
        //Setting goal_found to false is redundant because goal_receive is always true after first goal received
        //And generate goal is always called to generate goal to set goal_found before anywhere uses the goal
        // goal_found = false; 

        // goal_receive = false;
    }

    void shared_DWANode::checkDestination()
    {
        if (global_receive)
        {
            float dist2global = shared_DWANode::calDistance(0, 0, global_goal.position.x, global_goal.position.y);
            // ROS_INFO_STREAM("dist2global " << dist2global);
            // ROS_INFO_STREAM("min_dist2global " << min_dist2global);
            if (dist2global <= 1.5 || min_dist2global <= 0.5)
            {
                cancel_publisher_.publish(dummy_ID);
                global_receive = false;
                min_dist2global = 100;
                // ROS_INFO_STREAM("move_base canceling.");
            }
        }
    }

    bool shared_DWANode::dynamicWindow()
    {
        v_agent = std::max(std::min(v_max_robot, v_agent), -v_max_robot);
        w_agent = std::max(std::min(w_max_robot, w_agent), -w_max_robot);

        // compute the boudaries of dynamic window
        agent_forward = true;
        if (v_cmd < 0)
            agent_forward = false;

        float v_upper = std::min(v_agent + v_acclrt * dynamic_window_time_interval, v_max);
        float v_lower = std::max(v_agent - v_acclrt * dynamic_window_time_interval, -v_max);
        float w_left = std::min(w_agent + w_acclrt * dynamic_window_time_interval, w_max);
        float w_right = std::max(w_agent - w_acclrt * dynamic_window_time_interval, -w_max);

        //If dynamic window crosses the 0 line, prune window to v_agent
        if(v_upper * v_lower < 0)
        {
            if(v_cmd > 0)
                v_lower = std::min(v_agent, (float)0.0);
            else if(v_cmd < 0)
                v_upper = std::max(v_agent, (float)0.0);
        }

        // discretize the dynamic window w.r.t. v_sample and w_sample
        float row_increment = fabs(v_upper - v_lower) / static_cast<float>(v_sample);
        float col_increment = fabs(w_right - w_left) / static_cast<float>(w_sample);

        // initialize the dynamic window and cost window
        // dynamic window channel 1 is linear velocity
        // dynamic window channel 2 is angular velocity
        // cost window is the cost of all samples
        dynamic_window.create(v_sample + 1, w_sample + 1, CV_32FC2);
        cost_window.create(v_sample + 1, w_sample + 1, CV_32F);
        clearance_window.create(v_sample + 1, w_sample + 1, CV_32F);
        heading_window.create(v_sample + 1, w_sample + 1, CV_32F);
        velocity_window.create(v_sample + 1, w_sample + 1, CV_32F);

        // goal_based navigation
        if (goal_found)
        {
            path_cost_window.create(v_sample + 1, w_sample + 1, CV_32F);
            dist_cost_window.create(v_sample + 1, w_sample + 1, CV_32F);
            angle_cost_window.create(v_sample + 1, w_sample + 1, CV_32F);
        }

        //Split dyanamic window evaluation into threads
        int threads = std::thread::hardware_concurrency();
        //Limit number of threads to v_sample
        if(threads > v_sample)
            threads = v_sample;

        //Create containers for async futures and visualization markers
        std::vector<std::future<void>> future_vector;
        std::vector<std::vector<geometry_msgs::Point>> candidate_samples_vec(threads);
        future_vector.reserve(threads);
        int v_samples_per_thread = round(static_cast<float>(v_sample) / threads);

        //Start threads with async lambda function
        //Each thread accesses a different part of all the windows, therefore no thread-safe mechanisms are required to prevent race conditions
        for (int k = 0; k < threads; ++k)
        {
            //This section is the computation done to process each velocity pair
            //If additional processing is to be done to for each velocity pair, ensure that the function called does not modify a global resource
            future_vector.emplace_back(std::async(std::launch::async, [&, k]() {
                int start = k * v_samples_per_thread;
                int end = start + v_samples_per_thread;

                //Last thread to take up all remainder samples
                if(k == threads - 1)
                    end = v_sample + 1;

                for (int i = start; i < end; ++i)
                {
                    for (int j = 0; j < w_sample + 1; ++j)
                    {
                        float v_dw = v_upper - i * row_increment;
                        float w_dw = w_left - j * col_increment;
                        float angle2goal = goal_yaw;
                        float min_dist2goal = calDistance(0, 0, dwa_goal.pose.position.x, dwa_goal.pose.position.y);
                        v_dw = round(v_dw * 1000) / 1000;
                        w_dw = round(w_dw * 1000) / 1000;

                        dynamic_window.at<cv::Vec2f>(i, j)[0] = v_dw;
                        dynamic_window.at<cv::Vec2f>(i, j)[1] = w_dw;

                        clearance_window.at<float>(i, j) = shared_DWANode::calDist2Collision(v_dw, w_dw, min_dist2goal, angle2goal, candidate_samples_vec[k]);
                        if (goal_found)
                        {
                            dist_cost_window.at<float>(i, j) = min_dist2goal;
                            angle_cost_window.at<float>(i, j) = angle2goal;
                        }
                        heading_window.at<float>(i, j) = shared_DWANode::calHeading(v_dw, w_dw, v_cmd, w_cmd);

                        if (fabs(v_cmd) == 0)
                            velocity_window.at<float>(i, j) = 0;
                        else
                            velocity_window.at<float>(i, j) = fabs(v_dw - v_cmd) / fabs(v_cmd);
                    }
                }
            }));
        }

        //Wait for all threads to complete
        for(auto & future : future_vector)
            future.get();

        if(enable_visualization)
        {
            for(const auto & samples_vec : candidate_samples_vec)
                candidate_samples.points.insert(candidate_samples.points.end(), samples_vec.begin(), samples_vec.end());
        }

        // ROS_DEBUG_STREAM("Window generated.");
        // ROS_DEBUG_STREAM("dist_cost_window " << dist_cost_window);
        // ROS_DEBUG_STREAM("angle_cost_window " << angle_cost_window);
        return true;
    }

    float shared_DWANode::calDist2Collision(float v_dw, float w_dw, float &min_dist2goal, float &angle2goal, std::vector<geometry_msgs::Point>& candidate_points)
    {
        //Fixed distance between trajectory projection points, minimum 5 samples
        double sample_interval = sample_distance / fabs(v_dw);
        if(sample_time / sample_interval < 5)
            sample_interval = sample_time / 5;
        int sample_number = ceil(sample_time / sample_interval);

        float dist2collision = 1;
        float x = 0, y = 0, theta = 0;
        if (v_dw == 0)
        {
            theta = w_dw * sample_interval;
            if (shared_DWANode::checkCollision(0, 0, theta, footprint_mode))
                dist2collision = 0;
        }

        else
        {
            //Handle current robot's position, if collide, return 0
            if (checkCollision(x, y, theta, footprint_mode))
                return 0;

            for (int i = 1; i < sample_number; i++)
            {
                if (w_dw == 0)
                {
                    x += v_dw * cos(theta) * sample_interval;
                    y += v_dw * sin(theta) * sample_interval;
                    theta = 0;
                }
                else
                {
                    x += v_dw / w_dw * (sin(theta + w_dw * sample_interval) - sin(theta));
                    y += v_dw / w_dw * (cos(theta) - cos(theta + w_dw * sample_interval));
                    theta += w_dw * sample_interval;
                }

                //Check only if the time exceeds collision_check_interval, and last point
                if (checkCollision(x, y, theta, footprint_mode))
                {
                    float linear_dist = (i + 1) * sample_interval * v_dw;
                    float angular_dist = (i + 1) * sample_interval * w_dw;
                    float v_collision = sqrt(2 * fabs(v_acclrt) * fabs(linear_dist));
                    float w_collision = sqrt(2 * fabs(w_acclrt) * fabs(angular_dist));
                    if (fabs(v_dw) >= v_collision || (w_dw != 0 && fabs(w_dw) >= w_collision))
                    {
                        dist2collision = 0;
                    }
                    else
                    {
                        dist2collision = i / (static_cast<float>(sample_number) - 1);
                    }

                    break;
                }

                if(enable_visualization)
                {
                    geometry_msgs::Point point_candidate;
                    point_candidate.x = x;
                    point_candidate.y = y;
                    point_candidate.z = 0;
                    candidate_points.push_back(std::move(point_candidate));
                }

                // goal_based navigation
                if (goal_found)
                {
                    float temp_dist2goal = calDistance(x, y, dwa_goal.pose.position.x, dwa_goal.pose.position.y);
                    // temp_dist2goal = (temp_dist2goal >= 0.2) ? temp_dist2goal : 0.2;
                    if (temp_dist2goal <= min_dist2goal)
                    {
                        min_dist2goal = temp_dist2goal;
                        angle2goal = fabs(theta - goal_yaw);
                    }
                }
                if (global_receive)
                {
                    float temp_dist2global = calDistance(x, y, global_goal.position.x, global_goal.position.y);
                    if (temp_dist2global <= min_dist2global)
                    {
                        min_dist2global = temp_dist2global;
                    }
                }
            }
        }

        return dist2collision;
    }

    bool shared_DWANode::checkCollision(float x_check, float y_check, float theta_check, std::string footprint_mode)
    {
        if (footprint_mode == "radius")
        {
            for (const auto &point : lidar_points)
            {
                //https://stackoverflow.com/a/7227057
                //Check through easy conditions first, getting distance with sqrt is computationally expensive
                double dx = fabs(point.x - x_check);
                double dy = fabs(point.y - y_check);

                if (dx > r_collision || dy > r_collision)
                    continue;

                if (dx + dy <= r_collision || sqrtf(powf(dx, 2) + powf(dy, 2)) <= r_collision)
                    return true;
            }
            return false;
        }

        else if (footprint_mode == "rectangle")
        {
            // https://math.stackexchange.com/a/190373
            cv::Point2f A(0, 0), B(0, 0), C(0, 0), D(0, 0), M(0, 0);
            double cos_scale = cos(theta_check), sin_scale = sin(theta_check);
            A.x = x_check + rectangle_point[0].x * cos_scale - rectangle_point[0].y * sin_scale;
            A.y = y_check + rectangle_point[0].y * cos_scale + rectangle_point[0].x * sin_scale;
            B.x = x_check + rectangle_point[1].x * cos_scale - rectangle_point[1].y * sin_scale;
            B.y = y_check + rectangle_point[1].y * cos_scale + rectangle_point[1].x * sin_scale;
            C.x = x_check + rectangle_point[2].x * cos_scale - rectangle_point[2].y * sin_scale;
            C.y = y_check + rectangle_point[2].y * cos_scale + rectangle_point[2].x * sin_scale;
            D.x = x_check + rectangle_point[3].x * cos_scale - rectangle_point[3].y * sin_scale;
            D.y = y_check + rectangle_point[3].y * cos_scale + rectangle_point[3].x * sin_scale;

            cv::Vec2f AB(B.x - A.x, B.y - A.y);
            cv::Vec2f AD(D.x - A.x, D.y - A.y);

            float dotABAB = calDotproduct(AB, AB); //squared magnitude of AB
            float dotADAD = calDotproduct(AD, AD); //squared magnitude of AD

            for (const auto &point : lidar_points)
            {
                M.x = point.x;
                M.y = point.y;

                cv::Vec2f AM(M.x - A.x, M.y - A.y);
                // cv::Vec2f BM(M.x - B.x, M.y - B.y);

                float dotAMAB = calDotproduct(AM, AB);
                float dotAMAD = calDotproduct(AM, AD);

                if (0 <= dotAMAB && dotAMAB <= dotABAB && 0 <= dotAMAD && dotAMAD <= dotADAD)
                {
                    return true;
                    break;
                }
            }
            return false;
        }
        else
        {
            ROS_ERROR_STREAM("Not a processible footprint, please either use \"radius\" or \"rectangle\"! ");
            return true;
        }
    }

    float shared_DWANode::calDistance(float x1, float y1, float x2, float y2)
    {
        return sqrt(powf((x1 - x2), 2) + powf((y1 - y2), 2));
    }

    float shared_DWANode::calDotproduct(cv::Vec2f v1, cv::Vec2f v2)
    {
        return v1[0] * v2[0] + v1[1] * v2[1];
    }

    // calculate the heading value
    float shared_DWANode::calHeading(float v_current, float w_current, float v_command, float w_command)
    {
        // ROS_INFO_STREAM("agent_forward " <<  agent_forward);
        // v_current = (2 * static_cast<float>(agent_forward) - 1) * std::max( fabs(v_current / v_max_robot), static_cast<float>(0.0001));
        // w_current = (2 * static_cast<float>(agent_forward) - 1) * w_current / w_max_robot;
        // v_command = v_command / v_max_robot;
        // if (v_command >= 0)
        // {
        //     w_command = w_command / w_max_robot;
        // }
        // else
        // {
        //     w_command = - w_command / w_max_robot;
        // }
        // ROS_INFO_STREAM("v_command " <<  v_command << ", w_command " << w_command);
        if (v_current == 0)
        {
            v_current = 0.0001;
        }
        else
        {
            v_current = v_current / v_max_robot;
        }
        w_current = w_current / w_max_robot;
        v_command = v_command / v_max_robot;
        w_command = w_command / w_max_robot;
        tf2::Vector3 V_current(v_current, w_current, 0);
        tf2::Vector3 V_command(v_command, w_command, 0);
        // ROS_INFO_STREAM("V_command.X " <<  V_command.getX() << ", V_command.Y " << V_command.getY());
        // ROS_INFO_STREAM("V_current.X " <<  V_current.getX() << ", V_current.Y " << V_current.getY());
        return V_current.angle(V_command) / M_PI;
    }

    void shared_DWANode::selectVelocity()
    {
        float weight_goal_local, weight_cmd_local, v_optimal_cmd, w_optimal_cmd, v_optimal_path, w_optimal_path;
        cv::Mat cmd_cost_window = weight_heading * heading_window + weight_velocity * velocity_window;
        // cv::normalize(cmd_cost_window, cmd_cost_window, 0.0, 1.0, cv::NORM_MINMAX);
        // cv::pow(cmd_cost_window, gamma_cmd, cmd_cost_window);
        if (goal_found && agent_forward && fabs(v_cmd) > 0.2)
        {
            weight_goal_local = weight_goal;
            weight_cmd_local = weight_cmd;
            path_cost_window = shared_DWANode::calPathcost(dist_cost_window, angle_cost_window);
            // cv::normalize(path_cost_window, path_cost_window, 0.0, 1.0, cv::NORM_MINMAX);
            if (enable_dynamic_weight)
            {
                float cmd_discount = powf(cv::sum(clearance_window)[0] / ((v_sample + 1) * (w_sample + 1)), 2.5);
                weight_cmd_local = weight_cmd_local * cmd_discount;
                weight_cmd_local = (weight_cmd_local >= weight_cmd_lb) ? weight_cmd_local : weight_cmd_lb;
                weight_goal_local = 1 - weight_cmd_local;
            }
            // std::cout << "Weight_goal " << weight_goal_local << std::endl;

            // Get min values of cmd cost and path cost
            int min_index[2] = {};
            cv::minMaxIdx(cmd_cost_window, NULL, NULL, min_index, NULL);
            v_optimal_cmd = dynamic_window.at<cv::Vec2f>(min_index[0], min_index[1])[0];
            w_optimal_cmd = dynamic_window.at<cv::Vec2f>(min_index[0], min_index[1])[1];
            std::fill( std::begin(min_index), std::end(min_index), 0);
            cv::minMaxIdx(path_cost_window, NULL, NULL, min_index, NULL);
            v_optimal_path = dynamic_window.at<cv::Vec2f>(min_index[0], min_index[1])[0];
            w_optimal_path = dynamic_window.at<cv::Vec2f>(min_index[0], min_index[1])[1];

            // std::cout << "v_optimal_cmd " << v_optimal_cmd << ",w_optimal_cmd " << w_optimal_cmd << std::endl;
            // std::cout << "v_optimal_path " << v_optimal_path << ",w_optimal_path " << w_optimal_path << std::endl;
            float v_combined = weight_cmd_local * v_optimal_cmd + weight_goal_local * v_optimal_path;
            float w_combined = weight_cmd_local * w_optimal_cmd + weight_goal_local * w_optimal_path;
            // std::cout << "v_combined " << v_combined << ",w_combined " << w_combined << std::endl;

            for (int i = 0; i < v_sample + 1; ++i)
            {
                for (int j = 0; j < w_sample + 1; ++j)
                {
                    cost_window.at<float>(i, j) = weight_heading * 
                        shared_DWANode::calHeading(dynamic_window.at<cv::Vec2f>(i, j)[0], dynamic_window.at<cv::Vec2f>(i, j)[1], v_combined, w_combined)
                        + weight_velocity * 
                        (v_combined == 0 ? 0 : fabs(dynamic_window.at<cv::Vec2f>(i, j)[0] - v_combined) / fabs(v_combined));
                }
            }
            // cost_window = weight_cmd_local * cmd_cost_window + weight_goal_local * path_cost_total;
        }
        else
        {
            path_cost_window.release();
            // path_cost_window.zeros(cost_window.rows, cost_window.cols, CV_32F);
            weight_cmd_local = 1;
            cost_window = cmd_cost_window;
        }
        final_cmd_weight.data = weight_cmd_local;

        //Check if normalizing for clearance window is required. Might be required when obstacle is in robot's current footprint
        double max_clearance;
        cv::minMaxIdx(clearance_window, NULL, &max_clearance, NULL, NULL);
        if(max_clearance > 1.0)
            cv::normalize(clearance_window, clearance_window, 0.0, 1.0, cv::NORM_MINMAX);

        //Calculate final cost
        cost_window = 1 - clearance_window + clearance_window.mul(cost_window);
        cmd_cost_window = 1 - clearance_window + clearance_window.mul(cmd_cost_window);

        //Get min cost
        double min_cost = 0;
        int min_idx[2] = {};
        cv::minMaxIdx(cost_window, &min_cost, NULL, min_idx, NULL);

        //If min cost is a velocity pair that has clearance less than threshold, reject velocity
        if (clearance_window.at<float>(min_idx[0], min_idx[1]) <= min_clearance_threshold)
        {
            dwa_twist.linear.x = v_cmd;
            dwa_twist.angular.z = w_cmd;
            if (v_cmd > 0.15)
            {
                dwa_twist.linear.x = 0.1;
            }
            else if (v_cmd < -0.15)
            {
                dwa_twist.linear.x = -0.1;
            }
            // dwa_twist.linear.x = (fabs(dwa_twist.linear.x) > 0.2) ? 0.2 : dwa_twist.linear.x;  
            // dwa_twist.angular.z = 0;
            ROS_WARN_STREAM("Wheelchair inside collision zone now!");

            collision_marker_publisher_.publish(collision_marker_);
        }
        else
        {
            dwa_twist.linear.x = dynamic_window.at<cv::Vec2f>(min_idx[0], min_idx[1])[0];
            dwa_twist.angular.z = dynamic_window.at<cv::Vec2f>(min_idx[0], min_idx[1])[1];
        }

        std::fill( std::begin(min_idx), std::end(min_idx), 0);
        cv::minMaxIdx(cmd_cost_window, NULL, NULL, min_idx, NULL);
        v_optimal_cmd = dynamic_window.at<cv::Vec2f>(min_idx[0], min_idx[1])[0];
        w_optimal_cmd = dynamic_window.at<cv::Vec2f>(min_idx[0], min_idx[1])[1];

        if (clearance_window.at<float>(min_idx[0], min_idx[1]) <= min_clearance_threshold)
        {
            pure_dwa_twist.linear.x = v_cmd;
            pure_dwa_twist.angular.z = w_cmd;
            if (v_cmd > 0.2)
            {
                pure_dwa_twist.linear.x = 0.2;
            }
            else if (v_cmd < -0.2)
            {
                pure_dwa_twist.linear.x = -0.2;
            }
        }
        else
        {
            pure_dwa_twist.linear.x = v_optimal_cmd;
            pure_dwa_twist.angular.z = w_optimal_cmd;
        }

        if (enable_visualization)
        {
            geometry_msgs::Point final_line_point, cmd_line_point, robot_line_point, udwa_line_point;
            final_line_point.x = cmd_line_point.x = robot_line_point.x = udwa_line_point.x = 0;
            final_line_point.y = cmd_line_point.y = robot_line_point.y = udwa_line_point.y = 0;
            final_line_point.z = cmd_line_point.z = robot_line_point.z = udwa_line_point.z = 0;
            float theta_final = 0, theta_cmd = 0, theta_robot = 0, theta_dwa = 0;

            //Hardcode sample interval and number for visualization purposes only
            double marker_lifetime = 2.0 * algorithm_interval;
            waypoint_txt.lifetime = ros::Duration(marker_lifetime);
            float sample_interval = 0.15;
            float sample_number = sample_time / sample_interval;
            std::string line_type;
            std::stringstream cmd_weight_stream;
            for (int i = 0; i < sample_number; i++)
            {
                final_line_point.x += dwa_twist.linear.x * cos(theta_final) * sample_interval;
                final_line_point.y += dwa_twist.linear.x * sin(theta_final) * sample_interval;
                theta_final += dwa_twist.angular.z * sample_interval;
                final_line.points.push_back(final_line_point);
                cmd_line_point.x += v_cmd * cos(theta_cmd) * sample_interval;
                cmd_line_point.y += v_cmd * sin(theta_cmd) * sample_interval;
                theta_cmd += w_cmd * sample_interval;
                cmd_line.points.push_back(cmd_line_point);
                robot_line_point.x += v_optimal_path * cos(theta_robot) * sample_interval;
                robot_line_point.y += v_optimal_path * sin(theta_robot) * sample_interval;
                theta_robot += w_optimal_path * sample_interval;
                robot_line.points.push_back(robot_line_point);
                udwa_line_point.x += v_optimal_cmd * cos(theta_dwa) * sample_interval;
                udwa_line_point.y += v_optimal_cmd * sin(theta_dwa) * sample_interval;
                theta_dwa += w_optimal_cmd * sample_interval;
                udwa_line.points.push_back(udwa_line_point);
                waypoint_txt.color.r = 0;
                // if (i == round(sample_number * 2 / 3))
                // {
                //     line_type = "g-dwa";
                //     waypoint_txt.id = 6;
                //     waypoint_txt.text = line_type;
                //     waypoint_txt.pose.position = final_line_point;
                //     waypoint_txt.pose.orientation.w = 1;
                //     line_namelist.markers.push_back(waypoint_txt);
                //     line_type = "cmd";
                //     waypoint_txt.id = 7;
                //     waypoint_txt.text = line_type;
                //     waypoint_txt.pose.position = cmd_line_point;
                //     waypoint_txt.pose.orientation.w = 1;
                //     line_namelist.markers.push_back(waypoint_txt);
                //     // line_type = "auto";
                //     // waypoint_txt.id = 8;
                //     // waypoint_txt.text = line_type;
                //     // waypoint_txt.pose.position = robot_line_point;
                //     // waypoint_txt.pose.orientation.w = 1;
                //     // line_namelist.markers.push_back(waypoint_txt);
                //     line_type = "s-dwa";
                //     waypoint_txt.id = 9;
                //     waypoint_txt.text = line_type;
                //     waypoint_txt.pose.position = udwa_line_point;
                //     waypoint_txt.pose.orientation.w = 1;
                //     line_namelist.markers.push_back(waypoint_txt);
                // }
            }
            cmd_weight_stream << std::fixed << std::setprecision(2) << weight_cmd_local;
            line_type = cmd_weight_stream.str();
            waypoint_txt.id = 10;
            waypoint_txt.text = line_type;
            waypoint_txt.pose.position.x = -1;
            waypoint_txt.pose.position.y = 0;
            waypoint_txt.pose.orientation.w = 1;
            waypoint_txt.scale.z = 0.7;
            waypoint_txt.color.r = 1.0;
            waypoint_txt.color.g = 1.0;
            waypoint_txt.color.b = 1.0;
            waypoint_prob.markers.push_back(waypoint_txt);
            waypoint_txt.scale.z = 0.5;
            waypoint_txt.color.r = 0.0;
            waypoint_txt.color.g = 0.0;
            waypoint_txt.color.b = 0.0;
        }
        // if(enable_visualization)
        // {
        //     geometry_msgs::Point final_line_point, cmd_line_point, origin_dwa_point;
        //     final_line_point.x = cmd_line_point.x = origin_dwa_point.x = 0;
        //     final_line_point.y = cmd_line_point.y = origin_dwa_point.y = 0;
        //     final_line_point.z = cmd_line_point.z = origin_dwa_point.z = 0;
        //     float theta_final = 0, theta_cmd = 0, theta_dwa = 0;

        //     //Hardcode sample interval and number for visualization purposes only
        //     float sample_interval = 0.15;
        //     float sample_number = sample_time / sample_interval;
        //     for (int i = 0; i < sample_number; i++)
        //     {
        //         final_line_point.x += dwa_twist.linear.x * cos(theta_final) * sample_interval;
        //         final_line_point.y += dwa_twist.linear.x * sin(theta_final) * sample_interval;
        //         theta_final += dwa_twist.angular.z * sample_interval;
        //         final_line.points.push_back(final_line_point);
        //         origin_dwa_point.x += original_dwa_twist.linear.x * cos(theta_dwa) * sample_interval;
        //         origin_dwa_point.y += original_dwa_twist.linear.x * sin(theta_dwa) * sample_interval;
        //         theta_dwa += original_dwa_twist.angular.z * sample_interval;
        //         dwa_line.points.push_back(origin_dwa_point);
        //         cmd_line_point.x += v_cmd * cos(theta_cmd) * sample_interval;
        //         cmd_line_point.y += v_cmd * sin(theta_cmd) * sample_interval;
        //         theta_cmd += w_cmd * sample_interval;
        //         cmd_line.points.push_back(cmd_line_point);
        //     }
        // }
        // // ROS_INFO_STREAM("linear " <<  dwa_twist.linear.x << ", angular " << dwa_twist.angular.z);
    }

    cv::Mat shared_DWANode::calPathcost(cv::Mat dist_cost, cv::Mat angle_cost)
    {
        // // ROS_DEBUG_STREAM("dist_cost " << dist_cost);
        // // ROS_DEBUG_STREAM("angle_cost " << angle_cost);
        // double max_dist_cost = 0, min_dist_cost = 0, max_angle_cost = 0, min_angle_cost = 0;
        // cv::minMaxIdx(dist_cost, &min_dist_cost, &max_dist_cost, NULL, NULL);
        // cv::minMaxIdx(angle_cost, &min_angle_cost, &max_angle_cost, NULL, NULL);
        // for (int i = 0; i < dist_cost.rows; i++)
        // {
        //     for (int j = 0; j < dist_cost.cols; j++)
        //     {
        //         dist_cost.at<float>(i, j) = (dist_cost.at<float>(i, j) - min_dist_cost) / (max_dist_cost - min_dist_cost);
        //         angle_cost.at<float>(i, j) = (angle_cost.at<float>(i, j) - min_angle_cost) / (max_angle_cost - min_angle_cost);
        //     }
        // }
        cv::normalize(dist_cost, dist_cost, 0.0, 1.0, cv::NORM_MINMAX);
        cv::normalize(angle_cost, angle_cost, 0.0, 1.0, cv::NORM_MINMAX);
        return dist_cost * weight_distance + angle_cost * (1 - weight_distance);
    }

    bool shared_DWANode::generateGoal()
    {
        //If goal is invalid, reset dwa_goal internally
        if(goal_data.poses.size() == 0.0)
        {
            dwa_goal.header.frame_id = "";
            return false;
        }
        else
        {
            belief_goal.clear();
            geometry_msgs::Point temp_waypoint;
            std::string prob_value;
            for (int i = 0; i < goal_data.poses.size(); ++i)
            {
                belief_goal.push_back(goal_data.poses[i].position.z);
            }
            std::vector<float>::iterator itMax = std::max_element(belief_goal.begin(), belief_goal.end());
            int goal_index = std::distance(belief_goal.begin(), itMax);
            double marker_lifetime = 2.0 * algorithm_interval;
            waypoint_txt.lifetime = ros::Duration(marker_lifetime);

            for (int i = 0; i < goal_data.poses.size(); ++i)
            {
                if (i == goal_index)
                {
                    waypoint_txt.color.r = 1;
                }
                else
                {
                    waypoint_txt.color.r = 0;
                }
                temp_waypoint = goal_data.poses[i].position;
                temp_waypoint.z = 0;
                prob_value = std::to_string(belief_goal[i]);
                prob_value = prob_value.substr(0, prob_value.find(".")+3);
                waypoint_txt.id = i + 10 + 2 * goal_data.poses.size();
                waypoint_txt.text = prob_value;
                waypoint_txt.pose.position = temp_waypoint;
                waypoint_txt.pose.orientation.w = 1;
                waypoint_prob.markers.push_back(waypoint_txt);
                // waypoint_viz.points.push_back(temp_waypoint);
            }
            // waypoint_viz.points.push_back( goal_data.poses[goal_index].position);
            dwa_goal.header.frame_id = base_frame_id;
            dwa_goal.pose = goal_data.poses[goal_index];
            dwa_goal.pose.position.z = 0;
            tf2::convert(dwa_goal.pose.orientation, goal_quat);
            tf2Scalar yaw, pitch, roll;
            tf2::Matrix3x3(goal_quat).getRPY(roll, pitch, yaw);
            goal_yaw = yaw;
            return true;
        }
    }
} // namespace shared_DWA
