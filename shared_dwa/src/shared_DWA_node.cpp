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
        : node_handle_(node_handle), tf_listener(tf_buffer)
    {
        if (!readParameters())
        {
            ROS_ERROR_STREAM("Could not load parameters.");
            ros::requestShutdown();
        }

        // Subscribers & Publishers
        scan_subscriber_ = node_handle_.subscribe("/scan", 1, &shared_DWANode::scanCallback, this);
        odom_subscriber_ = node_handle_.subscribe("/odom", 1, &shared_DWANode::odomCallback, this);

        if(use_dynamic_obstacles)
            obstacles_subscriber_ = node_handle_.subscribe("/obstacles", 1, &shared_DWANode::obstaclesCallback, this);

        cmd_subscriber_ = node_handle_.subscribe("/input_converter/cmd_vel", 1, &shared_DWANode::cmdCallback, this);

        //If guidance mode is goal, subscribe to waypoints published from belief node
        if (guidance_mode == "goal")
        {
            goal_subscriber_ = node_handle_.subscribe("/waypoint_distribution", 1, &shared_DWANode::goalCallback, this);
            status_subscriber_ = node_handle_.subscribe("/robot_in_inflation", 1, &shared_DWANode::statusCallback, this);
        }

        vel_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/shared_dwa/cmd_vel", 1);
        // goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/shared_dwa/goal", 1);
        cancel_publisher_ = node_handle_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
        timer_ = node_handle_.createTimer(ros::Duration(publish_interval), &shared_DWANode::timerCallback, this);

        //Visualization publishers
        cddt_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/path", 1);
        line_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/final_path", 1);
        ucmd_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/user_path", 1);
        rcmd_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/robot_path", 1);
        wypt_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/waypoints", 1);

        //Variables are initialized to 0 by default
        // // initialize the final published twist
        // dwa_twist.linear.x = 0;
        // dwa_twist.linear.y = 0;
        // dwa_twist.linear.z = 0;
        // dwa_twist.angular.x = 0;
        // dwa_twist.angular.y = 0;
        // dwa_twist.angular.z = 0;

        // initialize dwa_goal
        // dwa_goal.header.frame_id = base_frame_id;
        // dwa_goal.pose.position.x = 0;
        // dwa_goal.pose.position.y = 0;
        // dwa_goal.pose.position.z = 0;
        // dwa_goal.pose.orientation.x = 0;
        // dwa_goal.pose.orientation.y = 0;
        // dwa_goal.pose.orientation.z = 0;
        // dwa_goal.pose.orientation.w = 1;

        // calculate the total sample time
        // sample_time = std::max(- v_max_robot / v_acclrt, - w_max_robot / w_acclrt);

        // number of samples
        sample_number = floor((sample_time / sample_interval));

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
        final_line.color.r = 1.0;
        final_line.color.a = 1.0;

        user_line.header.frame_id = base_frame_id;
        user_line.ns = "visualization";
        user_line.action = visualization_msgs::Marker::ADD;
        user_line.pose.orientation.w = 1.0;
        user_line.id = 2;
        user_line.type = visualization_msgs::Marker::LINE_STRIP;
        user_line.scale.x = 0.05;
        user_line.color.b = 1.0;
        user_line.color.a = 1.0;

        robot_line.header.frame_id = base_frame_id;
        robot_line.ns = "visualization";
        robot_line.action = visualization_msgs::Marker::ADD;
        robot_line.pose.orientation.w = 1.0;
        robot_line.id = 3;
        robot_line.type = visualization_msgs::Marker::LINE_STRIP;
        robot_line.scale.x = 0.05;
        robot_line.color.g = 1.0;
        robot_line.color.a = 1.0;

        waypoint_viz.header.frame_id = base_frame_id;
        waypoint_viz.ns = "visualization";
        waypoint_viz.action = visualization_msgs::Marker::ADD;
        waypoint_viz.pose.orientation.w = 1.0;
        waypoint_viz.id = 4;
        waypoint_viz.type = visualization_msgs::Marker::SPHERE_LIST;
        waypoint_viz.scale.x = 0.5;
        waypoint_viz.scale.y = 0.5;
        waypoint_viz.scale.z = 0.5;
        waypoint_viz.color.r = 1.0;
        waypoint_viz.color.b = 1.0;
        waypoint_viz.color.a = 1.0;
    }

    //Public Member Functions

    //Private Member Functions
    bool shared_DWANode::readParameters()
    {
        ROS_INFO_STREAM("Loading parameters.....");
        if (!node_handle_.getParam("publish_interval", publish_interval))
            ROS_WARN_STREAM("Parameter publish_interval not set. Using default setting: " << publish_interval);
        if (!node_handle_.getParam("sample_interval", sample_interval))
            ROS_WARN_STREAM("Parameter sample_interval not set. Using default setting: " << sample_interval);
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
        if (!node_handle_.getParam("use_dynamic_obstacles", use_dynamic_obstacles))
            ROS_WARN_STREAM("Parameter use_dynamic_obstacles not set. Using default setting: " << use_dynamic_obstacles);
        if (!node_handle_.getParam("min_clearance_threshold", min_clearance_threshold))
            ROS_WARN_STREAM("Parameter min_clearance_threshold not set. Using default setting: " << min_clearance_threshold);
        if (!node_handle_.getParam("v_max_collision", v_max_collision))
            ROS_WARN_STREAM("Parameter v_max_collision not set. Using default setting: " << v_max_collision);
        if (!node_handle_.getParam("w_max_collision", w_max_collision))
            ROS_WARN_STREAM("Parameter w_max_collision not set. Using default setting: " << w_max_collision);
        if (!node_handle_.getParam("use_expected_cost", use_expected_cost))
            ROS_WARN_STREAM("Parameter use_expected_cost not set. Using default setting: " << use_expected_cost);

        //Footprint checking and processing
        if (footprint_mode == "radius")
        {
            if (!node_handle_.getParam("r_collision", r_collision))
                ROS_WARN_STREAM("Parameter r_collision not set. Using default setting: " << r_collision);
        }

        else if (footprint_mode == "rectangle")
        {
            rectangle_point.clear();
            std::vector<float> temp_list;
            cv::Point2f temp_point;
            if (node_handle_.getParam("rectangle_point", temp_list))
            {
                for (int i = 0; i < temp_list.size(); i += 2)
                {
                    temp_point.x = temp_list[i];
                    temp_point.y = temp_list[i + 1];
                    rectangle_point.push_back(temp_point);
                }
            }

            else
            {
                ROS_WARN_STREAM("Rectangle footprint not set, use default radius instead. Radius: " << r_collision);
                footprint_mode = "radius";
            }
        }

        else
        {
            ROS_ERROR_STREAM("The selected footprint_mode is not supported, please refer to the options in config file!");
            return false;
        }

        //Guidance mode checking
        if (guidance_mode == "disable")
            ROS_INFO_STREAM("Guidance mode disabled.");

        else if (guidance_mode == "goal")
            ROS_INFO_STREAM("Guidance mode selected to \"goal\".");

        else
        {
            ROS_ERROR_STREAM("The selected guidance_mode is not supported, please refer to the options in config file!");
            return false;
        }

        ROS_INFO_STREAM("Complete loading parameters.");
        return true;
    }

    void shared_DWANode::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr &msg)
    {
        //Copy all segment obstacles
        segment_obstacles = msg->segments;

        //Copy all circle obstacles
        circle_obstacles = msg->circles;

        //Convert all obstacles to base_frame if not in base frame
        geometry_msgs::TransformStamped odomToBaseTF;
        if (msg->header.frame_id != base_frame_id)
        {
            try
            {
                odomToBaseTF = tf_buffer.lookupTransform(base_frame_id, msg->header.frame_id, ros::Time(0), ros::Duration(tf_buffer_timeout));
    }
            catch (tf2::TransformException &Exception)
            {
                ROS_ERROR_STREAM(Exception.what());
    }

            for(auto & segment : segment_obstacles)
            {
                tf2::doTransform<geometry_msgs::Point>(segment.first_point, segment.first_point, odomToBaseTF);
                tf2::doTransform<geometry_msgs::Point>(segment.last_point, segment.last_point, odomToBaseTF);
    }

            for(auto & circle : circle_obstacles)
            {
                tf2::doTransform<geometry_msgs::Vector3>(circle.velocity, circle.velocity, odomToBaseTF);
                tf2::doTransform<geometry_msgs::Point>(circle.center, circle.center, odomToBaseTF);
            }
        }
    }

    void shared_DWANode::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_scan)
    {
        //Get static transform from lidar to base_link in case they are not in the same frame
        if (lidar2baseTransform.header.frame_id != base_frame_id && msg_scan->header.frame_id != base_frame_id)
        {
            ROS_INFO("LIDAR is not in base link frame and transform has not been found yet, finding transform");
            try
            {
                lidar2baseTransform = tf_buffer.lookupTransform(base_frame_id, msg_scan->header.frame_id, ros::Time(0), ros::Duration(tf_buffer_timeout));
                ROS_INFO("Transform found, all future scans received by shared_dwa will be transformed before being used for collision checking");
            }
            catch (tf2::TransformException &Exception)
            {
                ROS_ERROR("LIDAR tranform could not be found, shared_dwa may be incorrect");
                ROS_ERROR_STREAM(Exception.what());
            }
        }

        lidar_points.clear();
        lidar_points.reserve(msg_scan->ranges.size());
        for (int k = 0; k < msg_scan->ranges.size(); ++k)
        {
            geometry_msgs::Point temp_point;
            temp_point.x = msg_scan->ranges[k] * cos(msg_scan->angle_min + k * msg_scan->angle_increment);
            temp_point.y = msg_scan->ranges[k] * sin(msg_scan->angle_min + k * msg_scan->angle_increment);

            //If transform header is not empty
            if (!lidar2baseTransform.header.frame_id.empty())
                tf2::doTransform<geometry_msgs::Point>(temp_point, temp_point, lidar2baseTransform);

            lidar_points.emplace_back(std::move(temp_point));
        }
    }

    void shared_DWANode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom)
    {
        v_agent = msg_odom->twist.twist.linear.x;
        w_agent = msg_odom->twist.twist.angular.z;
    }

    void shared_DWANode::cmdCallback(const geometry_msgs::Twist::ConstPtr &msg_cmd)
    {
        v_cmd = msg_cmd->linear.x;
        w_cmd = msg_cmd->angular.z;
        candidate_samples.header.stamp = ros::Time::now();
        cmd_receive = true;
    }

    void shared_DWANode::goalCallback(const path_belief_update::WaypointDistribution::ConstPtr &msg_goal)
    {
        waypoint_belief.distribution.clear();
        waypoint_belief.waypoints.poses.clear();
        waypoint_belief = *msg_goal;
        goal_received = true;
    }

    void shared_DWANode::statusCallback(const std_msgs::Bool::ConstPtr &msg_status)
    {
        inside_inflation = msg_status->data;
    }

    void shared_DWANode::timerCallback(const ros::TimerEvent &)
    {
        if (cmd_receive)
        {
            if (guidance_mode == "goal")
            {
                //Goal receive is always true after first time
                if (goal_received)
                    goal_found = shared_DWANode::generateGoal(waypoint_belief);
                else
                    goal_found = false;
            }

            //If no joystick input
            if (v_cmd == 0 && w_cmd == 0)
            {
                dwa_twist.linear.x = 0;
                dwa_twist.angular.z = 0;
            }

            else
            {
                v_max = std::min(fabs(v_cmd), fabs(v_max_robot));
                if (v_cmd == 0.0)
                    w_max = std::min(fabs(w_cmd), fabs(w_max_robot));

                else
                    w_max = w_max_robot;

                //Generate dynamic window
                if (!shared_DWANode::dynamicWindow())
                {
                    ROS_ERROR_STREAM("Error! Dynamic window is not generated.");
                    return;
                }
                else
                {
                    shared_DWANode::selectVelocity();
                }
            }

            shared_DWANode::publishResults();
        }
    }

    void shared_DWANode::publishResults()
    {
        vel_publisher_.publish(dwa_twist);
        // goal_publisher_.publish(dwa_goal);
        if (enable_visualization)
        {
            cddt_publisher_.publish(candidate_samples);
            line_publisher_.publish(final_line);
            ucmd_publisher_.publish(user_line);
            rcmd_publisher_.publish(robot_line);
            wypt_publisher_.publish(waypoint_viz);
        }

        //Reset all containers
        candidate_samples.points.clear();
        final_line.points.clear();
        user_line.points.clear();
        robot_line.points.clear();
        waypoint_viz.points.clear();
        dynamic_window.release();
        // path_cost_window.release();
        // dist_cost_window.release();
        // angle_cost_window.release();
        path_cost_window.clear();
        dist_cost_window.clear();
        angle_cost_window.clear();
        clearance_window.release();
        heading_window.release();
        velocity_window.release();
        cmd_receive = false;
        //Setting goal_found to false is redundant because goal_receive is always true after first goal received
        //And generate goal is always called to generate goal to set goal_found before anywhere uses the goal
        // goal_found = false; 

        // goal_received = false;
    }

    bool shared_DWANode::dynamicWindow()
    {
        //Forward simulate all moving obstacles n (sample_number) times into the future, shape of vector is obstacles[i][n] where i = sample time step, n = circular obstacles
        forward_sim_obs = forwardSimulateObstacles(circle_obstacles, sample_number, sample_interval);

        //Limit agent velocity
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
            for (int i = 0; i < belief_goal.size(); i++)
            {
                path_cost_window.push_back(cv::Mat(v_sample + 1, w_sample + 1, CV_32F));
                dist_cost_window.push_back(cv::Mat(v_sample + 1, w_sample + 1, CV_32F));
                angle_cost_window.push_back(cv::Mat(v_sample + 1, w_sample + 1, CV_32F));
            }
            // path_cost_window.create(v_sample + 1, w_sample + 1, CV_32F);
            // dist_cost_window.create(v_sample + 1, w_sample + 1, CV_32F);
            // angle_cost_window.create(v_sample + 1, w_sample + 1, CV_32F);
        }
        // std::chrono::time_point<std::chrono::system_clock> user_start;
        // user_start = std::chrono::system_clock::now();

        //Split dyanamic window evaluation into threads
        int threads = std::thread::hardware_concurrency();
        //Limit number of threads to v_sample
        if (threads > v_sample)
            threads = v_sample;

        //Create containers for async futures and visualization markers
        std::vector<std::future<void>> future_vector;
        std::vector<std::vector<geometry_msgs::Point>> candidate_samples_vec(threads);
        future_vector.reserve(threads);
        int v_samples_per_thread = v_sample / threads;

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
                if (k == threads - 1)
                    end = v_sample + 1;

                for (int i = start; i < end; ++i)
                {
                    for (int j = 0; j < w_sample + 1; ++j)
                    {
                        float v_dw = v_upper - i * row_increment;
                        float w_dw = w_left - j * col_increment;
                        std::vector<float> angle2goal = goal_yaw;
                        std::vector<float> min_dist2goal;
                        min_dist2goal.resize(belief_goal.size(), 100.0);
                        v_dw = round(v_dw * 1000) / 1000;
                        w_dw = round(w_dw * 1000) / 1000;

                        dynamic_window.at<cv::Vec2f>(i, j)[0] = v_dw;
                        dynamic_window.at<cv::Vec2f>(i, j)[1] = w_dw;

                        clearance_window.at<float>(i, j) = shared_DWANode::calDist2Collision(v_dw, w_dw, min_dist2goal, angle2goal, candidate_samples_vec[k]);
                        if (goal_found)
                        {
                            for (int l = 0; l < belief_goal.size(); l++)
                            {
                                dist_cost_window[l].at<float>(i, j) = min_dist2goal[l];
                                angle_cost_window[l].at<float>(i, j) = angle2goal[l];
                            }
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
        for (auto &future : future_vector)
            future.get();
        // double time_cost = (std::chrono::system_clock::now() - user_start).count() / 1000000000.0;
        // std::cout << "Time cost " << time_cost << std::endl;
        if (enable_visualization)
        {
            for (const auto &samples_vec : candidate_samples_vec)
                candidate_samples.points.insert(candidate_samples.points.end(), samples_vec.begin(), samples_vec.end());
        }
        // for (int i = 0; i < dist_cost_window.size(); i++)
        // {
        //     ROS_INFO_STREAM("dist_cost_window " << i);
        //     ROS_INFO_STREAM(dist_cost_window[i]);
        // }
        return true;
    }

    float shared_DWANode::calDist2Collision(float v_dw, float w_dw, std::vector<float> &min_dist2goal, std::vector<float> &angle2goal, std::vector<geometry_msgs::Point> &candidate_points)
    {
        float dist2collision = 1;
        float x = 0, y = 0, theta = 0;
        float sum = 0;
        int num_points = 0;

        //Check for collision at the current position
        bool collide_now = use_dynamic_obstacles ? this->checkDynamicCollision(x, y, theta, footprint_mode, 0) : this->checkCollision(x, y, theta, footprint_mode);
        if(collide_now)
        {
            //Restrict velocity pairs to low velocity only by setting dist2collision 0 for high vels
            if(fabs(v_dw) > v_max_collision || fabs(w_dw) > w_max_collision)
                return 0;
        }

        for (int i = 0; i < sample_number; ++i)
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

            //Special case cost calculation if obstacle is in current position
            if(collide_now)
            {
                if(use_dynamic_obstacles)
                    calcDynamicObsInterference(x, y, theta, footprint_mode, i, sum, num_points);

                else
                    calcObsInterference(x, y, theta, footprint_mode, sum, num_points);
            }

            //Normal collision checking if there are no obstacles in current position
            else
            {
                //If collision happens at this forward simulated position
                bool collide = use_dynamic_obstacles ? this->checkDynamicCollision(x, y, theta, footprint_mode, i) : this->checkCollision(x, y, theta, footprint_mode);
                if (collide)
                {
                    float linear_dist = (i + 1) * sample_interval * v_dw;
                    float angular_dist = (i + 1) * sample_interval * w_dw;
                    float v_collision = sqrt(2 * fabs(v_acclrt) * fabs(linear_dist));
                    float w_collision = sqrt(2 * fabs(w_acclrt) * fabs(angular_dist));

                    //If velocity pair exceeds max velocity allowed before collision, set dist2collision to 0
                    if (fabs(v_dw) >= v_collision || (w_dw != 0 && fabs(w_dw) >= w_collision))
                        dist2collision = 0;

                    //Calculate number of intervals that are clear from collision 
                    //If sample number is 3, there are 3 samples and 2 intervals. 
                    //If sample 1 (index 0) collides, then num intervals clear = 0, clearance = 0%
                    //If sample 2 (index 1) collides, then num intervals clear = 1, clearance = 1/(samples - 1) = 50%
                    //If sample 3 (index 2) collides, then num intervals clear = 2, clearance = 100%
                    else
                        dist2collision = i / (static_cast<float>(sample_number) - 1);

                    break;
                }

                if (enable_visualization)
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
                    for (int j = 0; j < belief_goal.size(); j++)
                    {
                        float temp_dist2goal = calDistance(x, y, goal_list.poses[j].position.x, goal_list.poses[j].position.y);
                        // temp_dist2goal = (temp_dist2goal >= 0.2) ? temp_dist2goal : 0.2;
                        if (temp_dist2goal <= min_dist2goal[j])
                        {
                            min_dist2goal[j] = temp_dist2goal;
                            angle2goal[j] = fabs(theta - goal_yaw[j]);
                        }
                    }
                }

                if (global_received)
                {
                    float temp_dist2global = calDistance(x, y, global_goal.position.x, global_goal.position.y);
                    if (temp_dist2global <= min_dist2global)
                        min_dist2global = temp_dist2global;
                }
            }
        }

        if(collide_now)
            return sum/num_points;

        return dist2collision;
    }

    void shared_DWANode::calcDynamicObsInterference(float x_check, float y_check, float theta_check, const std::string &footprint_mode, int current_sample, float &sum, int &num_points)
    {
        std::vector<cv::Point2f> vertices(4, cv::Point2f(0, 0));
        cv::Vec2f AB, BC;
        //Calculate the 2 perpendicular sides of robot rotated to current heading
        if (footprint_mode == "rectangle")
        {
            double cos_scale = cos(theta_check), sin_scale = sin(theta_check);
            for(int i = 0; i < vertices.size(); ++i)
            {
                vertices[i].x = x_check + rectangle_point[i].x * cos_scale - rectangle_point[i].y * sin_scale;
                vertices[i].y = y_check + rectangle_point[i].y * cos_scale + rectangle_point[i].x * sin_scale;
            }

            AB = cv::Vec2f(vertices[1].x - vertices[0].x, vertices[1].y - vertices[0].y);
            BC = cv::Vec2f(vertices[2].x - vertices[1].x, vertices[2].y - vertices[1].y);
        }

        //Check through all line segments
        double squared_r_collision = pow(r_collision, 2);
        for (const auto &seg : segment_obstacles)
        {
            if (footprint_mode == "radius")
            {
                //Check if min distance of footprint center to line segment < radius
                double projection_dist = minDistPointToSegment(cv::Vec2f(seg.first_point.x, seg.first_point.y),
                                                               cv::Vec2f(seg.last_point.x, seg.last_point.y),
                                                               cv::Vec2f(x_check, y_check));

                if (projection_dist <= r_collision)
                {
                    num_points++;
                    sum += projection_dist;
                }
            }

            else if (footprint_mode == "rectangle")
            {
                for (int i = 0; i < vertices.size(); ++i)
                {
                    if(doIntersect(vertices[i], vertices[(i + 1) % 4], cv::Point2f(seg.first_point.x, seg.first_point.y), cv::Point2f(seg.last_point.x, seg.last_point.y)))
                    {
                        //If line segment intersects, find the closest distance between center of robot and line segment obstacle
                        double projection_dist = minDistPointToSegment(cv::Vec2f(seg.first_point.x, seg.first_point.y),
                                                                    cv::Vec2f(seg.last_point.x, seg.last_point.y),
                                                                    cv::Vec2f(x_check, y_check));
                        num_points++;
                        sum += projection_dist;
                    }
                }
            }
        }

        //Check through all circle obstacles that were forward simulated
        if (current_sample < forward_sim_obs.size())
        {
            for (const auto &obs : forward_sim_obs[current_sample])
            {
                if (footprint_mode == "radius")
                {
                    //Check if centers of circles are less or equal total radius
                    //TODO: Change offset to be rosparam
                    float dist = calDistance(x_check, y_check, obs.center.x, obs.center.y);
                    if (dist <= r_collision + obs.true_radius + 0.2)
                    {
                        sum += dist;
                        num_points++;
                    }
                }

                else if (footprint_mode == "rectangle")
                {
                    //Checks if center of obstacle lies inside the footprint
                    float dotABAB = calDotproduct(AB, AB); //squared magnitude of AB
                    float dotBCBC = calDotproduct(BC, BC); //squared magnitude of BC

                    cv::Point2f M(obs.center.x, obs.center.y);

                    cv::Vec2f AM(M.x - vertices[0].x, M.y - vertices[0].y);
                    cv::Vec2f BM(M.x - vertices[1].x, M.y - vertices[1].y);

                    float dotABAM = calDotproduct(AB, AM);
                    float dotBCBM = calDotproduct(BC, BM);

                    //Center of circle obstacle lies in rectangular footprint, get distance between the 2 centers
                    if (0 <= dotABAM && dotABAM <= dotABAB && 0 <= dotBCBM && dotBCBM <= dotBCBC)
                    {
                        float dist = calDistance(x_check, y_check, obs.center.x, obs.center.y);
                        sum += dist;
                        num_points++;
                    }

                    //Checks if obstacle intersects perimeter of footprint
                    for (int i = 0; i < vertices.size(); ++i)
                    {
                        double projection_dist = minDistPointToSegment(cv::Vec2f(vertices[i].x, vertices[i].y),
                                                                       cv::Vec2f(vertices[(i + 1) % 4].x, vertices[(i + 1) % 4].y),
                                                                       cv::Vec2f(obs.center.x, obs.center.y));
                                                                       
                        if (projection_dist <= obs.true_radius)
                        {
                            float dist = calDistance(x_check, y_check, obs.center.x, obs.center.y);
                            sum += dist;
                            num_points++;
                        }
                    }
                }
            }
        }
    }

    void shared_DWANode::calcObsInterference(float x_check, float y_check, float theta_check, const std::string &footprint_mode, float &sum, int &num_points)
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
                {
                    sum += powf(dx, 2) + powf(dy, 2);
                    num_points++;
                }
            }
            return;
        }

        else if (footprint_mode == "rectangle")
        {
            // https://math.stackexchange.com/a/190373
            cv::Point2f A(0, 0), B(0, 0), C(0, 0), M(0, 0);
            double cos_scale = cos(theta_check), sin_scale = sin(theta_check);
            A.x = x_check + rectangle_point[0].x * cos_scale - rectangle_point[0].y * sin_scale;
            A.y = y_check + rectangle_point[0].y * cos_scale + rectangle_point[0].x * sin_scale;
            B.x = x_check + rectangle_point[1].x * cos_scale - rectangle_point[1].y * sin_scale;
            B.y = y_check + rectangle_point[1].y * cos_scale + rectangle_point[1].x * sin_scale;
            C.x = x_check + rectangle_point[2].x * cos_scale - rectangle_point[2].y * sin_scale;
            C.y = y_check + rectangle_point[2].y * cos_scale + rectangle_point[2].x * sin_scale;

            cv::Vec2f AB(B.x - A.x, B.y - A.y);
            cv::Vec2f BC(C.x - B.x, C.y - B.y);

            float dotABAB = calDotproduct(AB, AB); //squared magnitude of AB
            float dotBCBC = calDotproduct(BC, BC); //squared magnitude of BC

            for (const auto &point : lidar_points)
            {
                M.x = point.x;
                M.y = point.y;

                cv::Vec2f AM(M.x - A.x, M.y - A.y);
                cv::Vec2f BM(M.x - B.x, M.y - B.y);

                float dotABAM = calDotproduct(AB, AM);
                float dotBCBM = calDotproduct(BC, BM);

                if (0 <= dotABAM && dotABAM <= dotABAB && 0 <= dotBCBM && dotBCBM <= dotBCBC)
                {
                    sum += powf(x_check - point.x, 2) + powf(y_check - point.y, 2);
                    num_points++;
                }
            }
            return;
        }
        else
        {
            ROS_ERROR_STREAM("Not a processible footprint, please either use \"radius\" or \"rectangle\"! ");
            return;
        }
    }

    bool shared_DWANode::checkCollision(float x_check, float y_check, float theta_check, const std::string &footprint_mode)
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
            cv::Point2f A(0, 0), B(0, 0), C(0, 0), M(0, 0);
            double cos_scale = cos(theta_check), sin_scale = sin(theta_check);
            A.x = x_check + rectangle_point[0].x * cos_scale - rectangle_point[0].y * sin_scale;
            A.y = y_check + rectangle_point[0].y * cos_scale + rectangle_point[0].x * sin_scale;
            B.x = x_check + rectangle_point[1].x * cos_scale - rectangle_point[1].y * sin_scale;
            B.y = y_check + rectangle_point[1].y * cos_scale + rectangle_point[1].x * sin_scale;
            C.x = x_check + rectangle_point[2].x * cos_scale - rectangle_point[2].y * sin_scale;
            C.y = y_check + rectangle_point[2].y * cos_scale + rectangle_point[2].x * sin_scale;

            cv::Vec2f AB(B.x - A.x, B.y - A.y);
            cv::Vec2f BC(C.x - B.x, C.y - B.y);

            float dotABAB = calDotproduct(AB, AB); //squared magnitude of AB
            float dotBCBC = calDotproduct(BC, BC); //squared magnitude of BC

            for (const auto &point : lidar_points)
            {
                M.x = point.x;
                M.y = point.y;

                cv::Vec2f AM(M.x - A.x, M.y - A.y);
                cv::Vec2f BM(M.x - B.x, M.y - B.y);

                float dotABAM = calDotproduct(AB, AM);
                float dotBCBM = calDotproduct(BC, BM);

                if (0 <= dotABAM && dotABAM <= dotABAB && 0 <= dotBCBM && dotBCBM <= dotBCBC)
                    return true;
            }
            return false;
        }
        else
        {
            ROS_ERROR_STREAM("Not a processible footprint, please either use \"radius\" or \"rectangle\"! ");
            return true;
        }
    }

    bool shared_DWANode::checkDynamicCollision(float x_check, float y_check, float theta_check, const std::string &footprint_mode, int current_sample)
    {
        std::vector<cv::Point2f> vertices(4, cv::Point2f(0, 0));
        cv::Vec2f AB, BC;
        if (footprint_mode == "rectangle")
        {
            double cos_scale = cos(theta_check), sin_scale = sin(theta_check);
            for(int i = 0; i < vertices.size(); ++i)
            {
                vertices[i].x = x_check + rectangle_point[i].x * cos_scale - rectangle_point[i].y * sin_scale;
                vertices[i].y = y_check + rectangle_point[i].y * cos_scale + rectangle_point[i].x * sin_scale;
            }

            AB = cv::Vec2f(vertices[1].x - vertices[0].x, vertices[1].y - vertices[0].y);
            BC = cv::Vec2f(vertices[2].x - vertices[1].x, vertices[2].y - vertices[1].y);
        }

        //Check through all line segments
        double squared_r_collision = pow(r_collision, 2);
        for (const auto &seg : segment_obstacles)
        {
            if (footprint_mode == "radius")
            {
                //Check if min distance of footprint center to line segment < radius
                double projection_dist = minDistPointToSegment(cv::Vec2f(seg.first_point.x, seg.first_point.y),
                                                               cv::Vec2f(seg.last_point.x, seg.last_point.y),
                                                               cv::Vec2f(x_check, y_check));

                if (projection_dist <= r_collision)
                    return true;
            }

            else if (footprint_mode == "rectangle")
            {
                for (int i = 0; i < vertices.size(); ++i)
                {
                    if(doIntersect(vertices[i], vertices[(i + 1) % 4], cv::Point2f(seg.first_point.x, seg.first_point.y), cv::Point2f(seg.last_point.x, seg.last_point.y)))
                        return true;
                }
            }
        }

        //Check through all circle obstacles that were forward simulated
        if (current_sample < forward_sim_obs.size())
        {
            for (const auto &obs : forward_sim_obs[current_sample])
            {
                if (footprint_mode == "radius")
                {
                    //Check if centers of circles are less or equal total radius
                    //TODO: Change offset to be rosparam
                    if (calDistance(x_check, y_check, obs.center.x, obs.center.y) <= r_collision + obs.true_radius + 0.2)
                        return true;
                }

                else if (footprint_mode == "rectangle")
                {
                    //Checks if center of obstacle lies inside the footprint
                    float dotABAB = calDotproduct(AB, AB); //squared magnitude of AB
                    float dotBCBC = calDotproduct(BC, BC); //squared magnitude of BC

                    cv::Point2f M(obs.center.x, obs.center.y);

                    cv::Vec2f AM(M.x - vertices[0].x, M.y - vertices[0].y);
                    cv::Vec2f BM(M.x - vertices[1].x, M.y - vertices[1].y);

                    float dotABAM = calDotproduct(AB, AM);
                    float dotBCBM = calDotproduct(BC, BM);

                    if (0 <= dotABAM && dotABAM <= dotABAB && 0 <= dotBCBM && dotBCBM <= dotBCBC)
                        return true;

                    //Checks if obstacle intersects perimeter of footprint
                    for (int i = 0; i < vertices.size(); ++i)
                    {
                        double projection_dist = minDistPointToSegment(cv::Vec2f(vertices[i].x, vertices[i].y),
                                                                       cv::Vec2f(vertices[(i + 1) % 4].x, vertices[(i + 1) % 4].y),
                                                                       cv::Vec2f(obs.center.x, obs.center.y));
                                                                       
                        if (projection_dist <= obs.true_radius)
                            return true;
                    }
                }
            }
        }

        return false;
    }

    float shared_DWANode::calDistance(float x1, float y1, float x2, float y2)
    {
        return sqrt(powf((x1 - x2), 2) + powf((y1 - y2), 2));
    }

    float shared_DWANode::squaredDistance(float x1, float y1, float x2, float y2)
    {
        return powf((x1 - x2), 2) + powf((y1 - y2), 2);
    }

    float shared_DWANode::calDotproduct(cv::Vec2f v1, cv::Vec2f v2)
    {
        return v1[0] * v2[0] + v1[1] * v2[1];
    }

    float shared_DWANode::minDistPointToSegment(const cv::Vec2f &v1, const cv::Vec2f &v2, const cv::Vec2f &p)
    {
        //https://stackoverflow.com/a/1501725
        float l2 = squaredDistance(v1[0], v1[1], v2[0], v2[1]);
        if (l2 == 0.0)
            return calDistance(v1[0], v1[1], v2[0], v2[1]);

        float t = std::max((float)0.0, std::min((float)1.0, calDotproduct(p - v1, v2 - v1) / l2));
        cv::Vec2f projection = v1 + t * (v2 - v1);

        return calDistance(p[0], p[1], projection[0], projection[1]);
    }

    bool shared_DWANode::onSegment(cv::Point2f p, cv::Point2f q, cv::Point2f r) 
    { 
        if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y)) 
            return true; 
    
        return false; 
    } 
    
    int shared_DWANode::orientation(cv::Point2f p, cv::Point2f q, cv::Point2f r) 
    { 
        double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y); 

        // colinear   
        if (val == 0.0) 
            return 0;   
    
        // clock or counterclock wise 
        return (val > 0.0) ? 1 : 2;   
    } 
    
    bool shared_DWANode::doIntersect(cv::Point2f p1, cv::Point2f q1, cv::Point2f p2, cv::Point2f q2) 
    { 
        // Find the four orientations needed for general and special cases 
        int o1 = orientation(p1, q1, p2); 
        int o2 = orientation(p1, q1, q2); 
        int o3 = orientation(p2, q2, p1); 
        int o4 = orientation(p2, q2, q1); 
    
        // General case 
        if (o1 != o2 && o3 != o4) 
            return true; 
    
        // Special Cases 
        // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
        if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
    
        // p1, q1 and q2 are colinear and q2 lies on segment p1q1 
        if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
    
        // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
        if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
    
        // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
        if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
    
        return false; // Doesn't fall in any of the above cases 
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
        if (goal_found && !inside_inflation && agent_forward && fabs(v_cmd) > 0.2)
        {
            weight_goal_local = weight_goal;
            weight_cmd_local = weight_cmd;
            if (enable_dynamic_weight)
            {
                float cmd_discount = powf(cv::sum(clearance_window)[0] / ((v_sample + 1) * (w_sample + 1)), 2.5);
                weight_cmd_local = weight_cmd_local * cmd_discount;
                weight_cmd_local = (weight_cmd_local >= weight_cmd_lb) ? weight_cmd_local : weight_cmd_lb;
                weight_goal_local = 1 - weight_cmd_local;
            }
            std::cout << "Weight_goal " << weight_goal_local << std::endl;
            cv::Mat path_cost_total;
            path_cost_total.zeros(cost_window.rows, cost_window.cols, CV_32F);
            // whether to use the most likely goal or the expected cost
            if (use_expected_cost)
            {
                double max_dist = 0, max_angle = 0, min_dist = 100, min_angle = 100;
                double temp_max_dist, temp_max_angle, temp_min_dist, temp_min_angle;
                for (int i = 0; i < belief_goal.size(); i++)
                {
                    cv::minMaxIdx(dist_cost_window[i], &temp_min_dist, &temp_max_dist, NULL, NULL);
                    cv::minMaxIdx(angle_cost_window[i], &temp_min_angle, &temp_max_angle, NULL, NULL);
                    if (temp_max_dist > max_dist)
                        max_dist = temp_max_dist;
                    if (temp_max_angle > max_angle)
                        max_angle = temp_max_angle;
                    if (temp_min_dist < min_dist)
                        min_dist = temp_min_dist;
                    if (temp_min_angle < min_angle)
                        min_angle = temp_min_angle;
                }
                for (int j = 0; j < belief_goal.size(); j++)
                {
                    path_cost_window[j] = (dist_cost_window[j] - min_dist) / (max_dist - min_dist);
                    // ROS_INFO_STREAM("dist_cost_window " << j);
                    // ROS_INFO_STREAM(path_cost_window[j]);
                    // path_cost_window[j] = weight_distance * path_cost_window[j] + (1 - weight_distance) * (angle_cost_window[j] - min_angle) / (max_angle - min_angle);
                    // path_cost_window[i] = shared_DWANode::calPathcost(dist_cost_window[i], angle_cost_window[i]);
                    // cv::normalize(path_cost_window[i], path_cost_window[i], 0.0, 1.0, cv::NORM_MINMAX);
                    // ROS_INFO_STREAM("dist_cost_window " << j << " belief " << belief_goal[j]);
                    // ROS_INFO_STREAM(path_cost_window[j]);
                    path_cost_total = path_cost_total + belief_goal[j] * path_cost_window[j];
                }
            }
            else
            {
                path_cost_total = shared_DWANode::calPathcost(dist_cost_window[max_index], angle_cost_window[max_index]);
            }
            // cv::normalize(path_cost_total, path_cost_total, 0.0, 1.0, cv::NORM_MINMAX);
            // ROS_INFO_STREAM("cmd_cost_window " << cmd_cost_window);
            // ROS_INFO_STREAM("path_cost_total " << path_cost_total);

            // Get min values of cmd cost and path cost
            int min_index[2] = {};
            cv::minMaxIdx(cmd_cost_window, NULL, NULL, min_index, NULL);
            v_optimal_cmd = dynamic_window.at<cv::Vec2f>(min_index[0], min_index[1])[0];
            w_optimal_cmd = dynamic_window.at<cv::Vec2f>(min_index[0], min_index[1])[1];
            std::fill( std::begin(min_index), std::end(min_index), 0);
            cv::minMaxIdx(path_cost_total, NULL, NULL, min_index, NULL);
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
            path_cost_window.clear();
            // path_cost_window.zeros(cost_window.rows, cost_window.cols, CV_32F);
            cost_window = cmd_cost_window;
        }

        //Check if normalizing for clearance window is required. Might be required when obstacle is in robot's current footprint
        double max_clearance;
        cv::minMaxIdx(clearance_window, NULL, &max_clearance, NULL, NULL);
        if(max_clearance > 1.0)
            cv::normalize(clearance_window, clearance_window, 0.0, 1.0, cv::NORM_MINMAX);

        //Calculate final cost
        cost_window = 1 - clearance_window + clearance_window.mul(cost_window);

        //Get min cost
        double min_cost = 0;
        int min_idx[2] = {};
        cv::minMaxIdx(cost_window, &min_cost, NULL, min_idx, NULL);

        //If min cost is a velocity pair that has clearance less than threshold, reject velocity
        if (clearance_window.at<float>(min_idx[0], min_idx[1]) <= min_clearance_threshold)
        {
            dwa_twist.linear.x = 0;
            dwa_twist.angular.z = 0;
        }
        else
        {
            dwa_twist.linear.x = dynamic_window.at<cv::Vec2f>(min_idx[0], min_idx[1])[0];
            dwa_twist.angular.z = dynamic_window.at<cv::Vec2f>(min_idx[0], min_idx[1])[1];
        }

        if (enable_visualization)
        {
            geometry_msgs::Point final_line_point, user_line_point, robot_line_point;
            final_line_point.x = user_line_point.x = robot_line_point.x = 0;
            final_line_point.y = user_line_point.y = robot_line_point.y = 0;
            final_line_point.z = user_line_point.z = robot_line_point.z = 0;
            float theta_final = 0, theta_user = 0, theta_robot = 0;
            for (int i = 0; i < sample_number; i++)
            {
                final_line_point.x += dwa_twist.linear.x * cos(theta_final) * sample_interval;
                final_line_point.y += dwa_twist.linear.x * sin(theta_final) * sample_interval;
                theta_final += dwa_twist.angular.z * sample_interval;
                final_line.points.push_back(final_line_point);
                user_line_point.x += v_cmd * cos(theta_user) * sample_interval;
                user_line_point.y += v_cmd * sin(theta_user) * sample_interval;
                theta_user += w_cmd * sample_interval;
                user_line.points.push_back(user_line_point);
                robot_line_point.x += v_optimal_path * cos(theta_robot) * sample_interval;
                robot_line_point.y += v_optimal_path * sin(theta_robot) * sample_interval;
                theta_robot += w_optimal_path * sample_interval;
                robot_line.points.push_back(robot_line_point);
            }
        }
        // ROS_INFO_STREAM("linear " <<  dwa_twist.linear.x << ", angular " << dwa_twist.angular.z);
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

    bool shared_DWANode::generateGoal(const path_belief_update::WaypointDistribution &msg_waypoint)
    {
        // check if the waypoint distribution is valid
        if (msg_waypoint.waypoints.poses.size() == 0 && msg_waypoint.distribution.size() == 0)
        {
            return false;
        }
        else
        {
            // dwa_goal.header.frame_id = base_frame_id;
            belief_goal = msg_waypoint.distribution;
            goal_list = msg_waypoint.waypoints;
            std::vector<float>::iterator max_iterator = std::max_element(belief_goal.begin(), belief_goal.end());
            max_index = max_iterator - belief_goal.begin();
            for (int i = 0; i < belief_goal.size(); i++)
            {
                tf2::convert(goal_list.poses[i].orientation, goal_quat);
                tf2Scalar yaw, pitch, roll;
                tf2::Matrix3x3(goal_quat).getRPY(roll, pitch, yaw);
                goal_yaw.push_back(yaw);
                float dist_single = calDistance(0, 0, goal_list.poses[i].position.x, goal_list.poses[i].position.y);
                dist2goal.push_back(dist_single);
                if (use_expected_cost)
                {
                    waypoint_viz.points.push_back(goal_list.poses[i].position);
                }
            }
            if (!use_expected_cost)
            {
                waypoint_viz.points.push_back(goal_list.poses[max_index].position);
            }
            // tf2::convert(dwa_goal.pose.orientation, goal_quat);
            // tf2Scalar yaw, pitch, roll;
            // tf2::Matrix3x3(goal_quat).getRPY(roll, pitch, yaw);
            // goal_yaw = yaw;
            return true;
        }
    }

    std::vector<std::vector<obstacle_detector::CircleObstacle>> shared_DWANode::forwardSimulateObstacles(const std::vector<obstacle_detector::CircleObstacle> &obstacles, int sample_number, double sample_interval)
    {
        // vec[i][n] means position of obstacle n at time i into the future
        std::vector<std::vector<obstacle_detector::CircleObstacle>> forward_sim_obs(sample_number, std::vector<obstacle_detector::CircleObstacle>(obstacles.size()));
        for (int obs = 0; obs < obstacles.size(); ++obs)
        {
            forward_sim_obs[0][obs] = obstacles[obs];
            for (int sample = 1; sample < sample_number; ++sample)
            {
                //Current sample's position is previous_position + interval * velocity
                forward_sim_obs[sample][obs].center.x = forward_sim_obs[sample - 1][obs].center.x + obstacles[obs].velocity.x * sample_interval;
                forward_sim_obs[sample][obs].center.y = forward_sim_obs[sample - 1][obs].center.y + obstacles[obs].velocity.y * sample_interval;
                forward_sim_obs[sample][obs].center.z = forward_sim_obs[sample - 1][obs].center.z + obstacles[obs].velocity.z * sample_interval;
                forward_sim_obs[sample][obs].true_radius = forward_sim_obs[sample - 1][obs].true_radius;
            }
        }

        return forward_sim_obs;
    }
} // namespace shared_DWA
