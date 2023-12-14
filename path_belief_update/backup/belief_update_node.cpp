#include "path_belief_update/belief_update_node.h"

namespace belief_update
{
    // Constructor
    belief_updateNode::belief_updateNode(ros::NodeHandle &node_handle)
    :   node_handle_(node_handle)
    {
        if (!readParameters())
        {
            ROS_ERROR("Could not load parameters.");
            ros::requestShutdown();
        }
        // Subscribers & Publishers
        odom_subscriber_ = node_handle_.subscribe("/odom", 1, &belief_updateNode::odomCallback, this);
        cmd_subscriber_ = node_handle_.subscribe("/input_converter/cmd_vel", 1, &belief_updateNode::cmdCallback, this);
        path_subscriber_ = node_handle_.subscribe("/all_paths", 1, &belief_updateNode::pathCallback, this);

        // vel_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/belief_update/cmd_vel", 1);
        goal_publisher_ = node_handle_.advertise<geometry_msgs::Pose>("/most_likely_goal", 1);
        noaction_timer_ = node_handle_.createTimer(ros::Duration(noaction_update_interval), &belief_updateNode::noactionTimerCallback, this);
        action_timer_ = node_handle_.createTimer(ros::Duration(action_update_interval), &belief_updateNode::actionTimerCallback, this);

        // std::vector<float> test_goal{0.1, 0.2, 0.3, 0.4, 0.0};
        // std::vector<float> soft_goal, clip_goal;
        // for (int i = 0; i < test_goal.size(); i++)
        // {
        //     ROS_INFO_STREAM("Test Goal " << i + 1 << ": " << test_goal[i]);
        // }
        // soft_goal = belief_updateNode::softMax(test_goal);
        // ROS_INFO_STREAM("After softmax:");
        // for (int i = 0; i < soft_goal.size(); i++)
        // {
        //     ROS_INFO_STREAM("Test Goal " << i + 1 << ": " << soft_goal[i]);
        // }
        // clip_goal = belief_updateNode::clipBelief(test_goal);
        // ROS_INFO_STREAM("After clipping:");
        // for (int i = 0; i < clip_goal.size(); i++)
        // {
        //     ROS_INFO_STREAM("Test Goal " << i + 1 << ": " << clip_goal[i]);
        // }
    }
    // Destructor
    belief_updateNode::~belief_updateNode()
    {}
    //Public Member Functions

    //Private Member Functions
    bool belief_updateNode::readParameters()
    {
        ROS_INFO_STREAM("Loading parameters.....");
        if (!node_handle_.getParam("noaction_update_interval", noaction_update_interval))
            ROS_WARN_STREAM("Parameter noaction_update_interval not set. Using default setting: " << noaction_update_interval);
        if (!node_handle_.getParam("action_update_interval", action_update_interval))
            ROS_WARN_STREAM("Parameter action_update_interval not set. Using default setting: " << action_update_interval);
        if (!node_handle_.getParam("time_horizon", time_horizon))
            ROS_WARN_STREAM("Parameter time_horizon not set. Using default setting: " << time_horizon);
        if (!node_handle_.getParam("path_frame_id", path_frame_id))
            ROS_WARN_STREAM("Parameter path_frame_id not set. Using default setting: " << path_frame_id);
        if (!node_handle_.getParam("base_frame_id", base_frame_id))
            ROS_WARN_STREAM("Parameter base_frame_id not set. Using default setting: " << base_frame_id);
        if (!node_handle_.getParam("odom_frame_id", odom_frame_id))
            ROS_WARN_STREAM("Parameter odom_frame_id not set. Using default setting: " << odom_frame_id);
        if (!node_handle_.getParam("discount_factor", discount_factor))
            ROS_WARN_STREAM("Parameter discount_factor not set. Using default setting: " << discount_factor);
        if (!node_handle_.getParam("dist_threshold", dist_threshold))
            ROS_WARN_STREAM("Parameter dist_threshold not set. Using default setting: " << dist_threshold);
        if (!node_handle_.getParam("dist_const_cost", dist_const_cost))
            ROS_WARN_STREAM("Parameter dist_const_cost not set. Using default setting: " << dist_const_cost);
        if (!node_handle_.getParam("angle_threshold", angle_threshold))
            ROS_WARN_STREAM("Parameter angle_threshold not set. Using default setting: " << angle_threshold);
        if (!node_handle_.getParam("angle_const_cost", angle_const_cost))
            ROS_WARN_STREAM("Parameter angle_const_cost not set. Using default setting: " << angle_const_cost);
        if (!node_handle_.getParam("weight_angle", weight_angle))
            ROS_WARN_STREAM("Parameter weight_angle not set. Using default setting: " << weight_angle);
        if (!node_handle_.getParam("weight_dist", weight_dist))
            ROS_WARN_STREAM("Parameter weight_dist not set. Using default setting: " << weight_dist);
        if (!node_handle_.getParam("tf_buffer_timeout", tf_buffer_timeout))
            ROS_WARN_STREAM("Parameter tf_buffer_timeout not set. Using default setting: " << tf_buffer_timeout);
        if (!node_handle_.getParam("waypoint_dist", waypoint_dist))
            ROS_WARN_STREAM("Parameter waypoint_dist not set. Using default setting: " << waypoint_dist);
        if (!node_handle_.getParam("temp_softmax", temp_softmax))
            ROS_WARN_STREAM("Parameter temp_softmax not set. Using default setting: " << temp_softmax);
        if (!node_handle_.getParam("upper_bound", upper_bound))
            ROS_WARN_STREAM("Parameter upper_bound not set. Using default setting: " << upper_bound);
        if (!node_handle_.getParam("lower_bound", lower_bound))
            ROS_WARN_STREAM("Parameter lower_bound not set. Using default setting: " << lower_bound);
        ROS_INFO_STREAM("Complete loading parameters.");
        return true;
    }

    void belief_updateNode::odomCallback(const nav_msgs::Odometry &msg_odom)
    {
        odom_frame_id = msg_odom.header.frame_id;
        // v_agent = msg_odom.twist.twist.linear.x;
        // w_agent = msg_odom.twist.twist.angular.z;
        // // v_agent = 0.6;
        // // w_agent = 0;
        // v_agent = round(v_agent * 10) / 10;
        // w_agent = round(w_agent * 10) / 10;
        agent_pose = msg_odom.pose.pose;
        // x_agent = msg_odom.pose.pose.position.x;
        // y_agent = msg_odom.pose.pose.position.y;
        // tf2::convert(msg_odom.pose.pose.orientation, agent_quat);
        odom_receive = true;
        // ROS_INFO_STREAM("Odom received.");
    }

    void belief_updateNode::cmdCallback(const geometry_msgs::Twist &msg_cmd)
    {
        v_cmd = msg_cmd.linear.x;
        w_cmd = msg_cmd.angular.z;
        v_cmd = round(v_cmd * 10) / 10;
        w_cmd = round(w_cmd * 10) / 10;
        if (v_cmd != 0 || w_cmd != 0)
        {
            action_update = true;
            
        }
        else
        {
            noaction_update = true;
        }
        
        cmd_receive = true;
        // ROS_INFO_STREAM("Command received.");
    }

    void belief_updateNode::pathCallback(const shared_voronoi_global_planner::PathList &msg_path)
    {
        path_list = msg_path;
        // check if the paths exist or have changed
        if (path_list.paths.size() == 0)
        {
            ROS_WARN_STREAM("Path candidates not received!");
            path_receive = false;
            return;
        }
        else
        {
            path_frame_id = path_list.paths[0].header.frame_id;
            if (belief_goal.size() == 0) // initialization
            {
                ROS_INFO_STREAM("Belief Space Initializing.");
                pre_path_list = path_list;
                belief_goal.resize(path_list.paths.size());
                for (int i = 0; i < path_list.paths.size(); i++)
                {
                    belief_goal[i] =  1 / static_cast<float>(belief_goal.size());
                }
            }
            else if (belief_goal.size() != path_list.paths.size()) // new paths have been found
            {
                ROS_INFO_STREAM("New paths added.");
                belief_goal.resize(path_list.paths.size());
                int num_changed_goal = 0;
                float sum_changed_goal = 0;
                for (int i = 0; i < path_list.paths.size(); i++)
                {
                    if (i < pre_path_list.paths.size()) // check if previous registered paths have changed
                    {
                        if (pre_path_list.paths[i].header.seq == path_list.paths[i].header.seq) // not changed
                        {
                            continue;
                        }
                        else // paths have been replaced
                        {
                            num_changed_goal++; // count the number of replaced paths
                            sum_changed_goal += belief_goal[i]; // sum of the probabilities over replaced paths
                            belief_goal[i] = 0; // assign zero probabilities to the replaced paths
                            ROS_INFO_STREAM( num_changed_goal << " path(s) has changed.");
                        }
                    }
                    else // assign zero probabilities to the appended new paths
                    {
                        belief_goal[i] = 0;
                    }
                }
                // distribute the probabilites from replaced paths to the remaining paths evenly
                for (int i = 0; i < path_list.paths.size(); i++)
                {
                    if (i < pre_path_list.paths.size() && pre_path_list.paths[i].header.seq == path_list.paths[i].header.seq)
                    {
                        belief_goal[i] += sum_changed_goal / (belief_goal.size() - num_changed_goal);
                    }
                }
                // store the current path info for the next checking process
                pre_path_list = path_list;

                belief_goal = belief_updateNode::softMax(belief_goal);
            }
            else // no new paths added, only replacement
            {
                // ROS_INFO_STREAM("Amount of paths not changed.");
                int num_changed_goal = 0;
                float sum_changed_goal = 0;
                for (int i = 0; i < path_list.paths.size(); i++)
                {
                    if (pre_path_list.paths[i].header.seq == path_list.paths[i].header.seq)
                    {
                        continue;
                    }
                    else
                    {
                        num_changed_goal++;
                        sum_changed_goal += belief_goal[i];
                        belief_goal[i] = 0;
                        ROS_INFO_STREAM( num_changed_goal << " path(s) has changed.");
                    }
                }
                for (int i = 0; i < path_list.paths.size(); i++)
                {
                    if (pre_path_list.paths[i].header.seq == path_list.paths[i].header.seq)
                    {
                        belief_goal[i] += sum_changed_goal / (belief_goal.size() - num_changed_goal);
                    }
                }
                pre_path_list = path_list;
                // ROS_INFO_STREAM("Before Softmax:");
                // for (int i = 0; i < belief_goal.size(); i++)
                // {
                //     ROS_INFO_STREAM("Belief Goal " << i + 1 << ": " << belief_goal[i]);
                // }
                belief_goal = belief_updateNode::softMax(belief_goal);
                // ROS_INFO_STREAM("After Softmax:");
                // for (int i = 0; i < belief_goal.size(); i++)
                // {
                //     ROS_INFO_STREAM("Belief Goal " << i + 1 << ": " << belief_goal[i]);
                // }
            }
            path_receive = true;
        }
    }

    void belief_updateNode::noactionTimerCallback(const ros::TimerEvent&)
    {
        if (noaction_update && odom_receive && path_receive)
        {
            belief_updateNode::generateGoal();
            belief_updateNode::updateGoalPrediction();
            belief_updateNode::publishResults();
            noaction_update = false;
        }
    }

    void belief_updateNode::actionTimerCallback(const ros::TimerEvent&)
    {
        if (action_update && odom_receive && path_receive)
        {
            belief_updateNode::generateGoal();
            belief_updateNode::updateGoalPrediction();
            belief_updateNode::publishResults();
            action_update = false;
        }
    }

    void belief_updateNode::generateGoal()
    {
        if (path_receive)
        {
            // transform from odom to map
            geometry_msgs::TransformStamped odom2mapTransform;
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener(tf_buffer);
            try
            {
                odom2mapTransform = tf_buffer.lookupTransform(path_frame_id, odom_frame_id, ros::Time(0), ros::Duration(tf_buffer_timeout));
            }
            catch (tf2::TransformException &Exception) 
            {
                ROS_ERROR_STREAM(Exception.what());
            }
            geometry_msgs::Pose agent_pose_map;
            tf2::doTransform<geometry_msgs::Pose>(agent_pose, agent_pose_map, odom2mapTransform);

            // geometry_msgs::Point point_marker;
            goal_list.poses.resize(path_list.paths.size());
            for (int i = 0; i < path_list.paths.size(); i++)
            {
                goal_list.poses[i] = belief_updateNode::findGoal(agent_pose_map, path_list.paths[i]);
                // point_marker.x = goal_list.poses[i].position.x;
                // point_marker.y = goal_list.poses[i].position.y;
                // point_marker.z = 0;
                // goal_marker.points.push_back(point_marker);
            }

            // transform from map to base_link
            geometry_msgs::TransformStamped map2localTransform;
            tf_buffer.clear();
            try
            {
                map2localTransform = tf_buffer.lookupTransform(base_frame_id, path_frame_id, ros::Time(0), ros::Duration(tf_buffer_timeout));
            }
            catch (tf2::TransformException &Exception) 
            {
                ROS_ERROR_STREAM(Exception.what());
            }
            for (int i = 0; i < goal_list.poses.size(); i++)
            {
                tf2::doTransform<geometry_msgs::Pose>(goal_list.poses[i], goal_list.poses[i], map2localTransform);
            }

            goal_list.header.frame_id = base_frame_id;
        }
    }

    geometry_msgs::Pose belief_updateNode::findGoal(const geometry_msgs::Pose &agent_pose, const nav_msgs::Path &msg_path)
    {
        geometry_msgs::Pose generated_goal;
        
        tf2::Vector3 goal_direction;
        tf2::Quaternion goal_quat;

        float acmlt_dist = belief_updateNode::calDistance(agent_pose.position.x, agent_pose.position.y,
            msg_path.poses[0].pose.position.x, msg_path.poses[0].pose.position.y);
        for (int i = 1; i < msg_path.poses.size(); i++)
        {
            if (i == msg_path.poses.size() - 1)
            {
                generated_goal = msg_path.poses[i].pose;
                goal_direction = tf2::Vector3(msg_path.poses[i].pose.position.x - msg_path.poses[i - 1].pose.position.x,
                    msg_path.poses[i].pose.position.y - msg_path.poses[i - 1].pose.position.y, 0);
                break;
            }
            acmlt_dist += belief_updateNode::calDistance(msg_path.poses[i - 1].pose.position.x, msg_path.poses[i - 1].pose.position.y,
            msg_path.poses[i].pose.position.x, msg_path.poses[i].pose.position.y);
            if (acmlt_dist >= waypoint_dist)
            {
                generated_goal = msg_path.poses[i].pose;
                goal_direction = tf2::Vector3(msg_path.poses[i].pose.position.x - msg_path.poses[i - 1].pose.position.x,
                    msg_path.poses[i].pose.position.y - msg_path.poses[i - 1].pose.position.y, 0);
                break;
            }
        }
        tf2Scalar yaw;
        yaw = goal_direction.angle(tf2::Vector3(1, 0, 0));
        if (goal_direction.getY() < 0)
        {
            yaw = -yaw;
        }
        goal_quat.setRPY(0, 0, yaw);
        tf2::convert(goal_quat, generated_goal.orientation);
        // ros::Time time_end = ros::Time::now();
        // ros::Duration duration = time_end - time_begin;
        // ROS_INFO_STREAM("Time Cost " << duration.toSec());
        return generated_goal;
    }

    void belief_updateNode::publishResults()
    {
        ROS_INFO_STREAM("After update:");
        for (int i = 0; i < belief_goal.size(); i++)
        {
            ROS_INFO_STREAM("Belief Goal " << i + 1 << ": " << belief_goal[i]);
        }
        std::vector<float>::iterator itMax = std::max_element(belief_goal.begin(), belief_goal.end());
        int goal_index = std::distance(belief_goal.begin(), itMax);
        goal_publisher_.publish(goal_list.poses[goal_index]);
        cmd_receive = false;
        odom_receive = false;
        path_receive = false;
        goal_list.poses.clear();
    }

    float belief_updateNode::calTranslationStateValue(geometry_msgs::Pose goal_pose)
    {
        return belief_updateNode::calDistance(0, 0, goal_pose.position.x, goal_pose.position.y);
    }

    float belief_updateNode::calRotationStateValue(geometry_msgs::Pose goal_pose)
    {
        tf2::Vector3 agent2goal(goal_pose.position.x, goal_pose.position.y, 0);
        tf2::Vector3 agent_heading(1, 0, 0);
        // float angle2goal = agent_heading.angle(agent2goal);
        return agent_heading.angle(agent2goal);
    }

    float belief_updateNode::calTranslationStateActionValue(float v_action, float w_action, geometry_msgs::Pose goal_pose)
    {
        float x_next, y_next, theta_next;
        if (w_action == 0)
        {
            x_next = v_action * time_horizon;
            y_next = 0;
            theta_next = 0;
        }
        else
        {
            x_next = (v_action / w_action) * (sin(w_action * time_horizon) - sin(0));
            y_next = (v_action / w_action) * (cos(0) - cos(w_action * time_horizon));
            theta_next = w_action * time_horizon;
        }
        float V_next_translation = belief_updateNode::calDistance(x_next, y_next, goal_pose.position.x, goal_pose.position.y);
        float C_action_translation = belief_updateNode::calTranslationActionCost(V_next_translation);
        // ROS_INFO_STREAM("C_action " << C_action << ", V_next " << V_next);
        return discount_factor * V_next_translation + C_action_translation;         
    }

    float belief_updateNode::calRotationStateActionValue(float v_action, float w_action, geometry_msgs::Pose goal_pose)
    {
        float x_next, y_next, theta_next;
        if (w_action == 0)
        {
            x_next = v_action * time_horizon;
            y_next = 0;
            theta_next = 0;
        }
        else
        {
            x_next = (v_action / w_action) * (sin(w_action * time_horizon) - sin(0));
            y_next = (v_action / w_action) * (cos(0) - cos(w_action * time_horizon));
            theta_next = w_action * time_horizon;
        }
        tf2::Vector3 agentnext2goal(goal_pose.position.x - x_next, goal_pose.position.y - y_next, 0);
        tf2::Vector3 agentnext_heading(1, 0, 0);
        tf2::Matrix3x3 rotation_matrix;
        rotation_matrix.setRPY(0, 0, theta_next);
        agentnext_heading = rotation_matrix * agentnext_heading;
        float V_next_rotation = agentnext_heading.angle(agentnext2goal);
        float C_action_rotation = belief_updateNode::calRotationActionCost(V_next_rotation);
        // ROS_INFO_STREAM("C_action " << C_action << ", V_next " << V_next);
        return discount_factor * V_next_rotation + C_action_rotation;
    }

    float belief_updateNode::calTranslationActionCost(float dis_next)
    {
        if (dis_next <= dist_threshold)
        {
            return dis_next * dist_const_cost / dist_threshold;
        }
        else
        {
            return dist_const_cost;
        }
    }

    float belief_updateNode::calRotationActionCost(float angle_next)
    {
        float angle_threshold_local = angle_threshold * PI / 180;
        if (angle_next <= angle_threshold_local)
        {
            return angle_next * angle_const_cost / angle_threshold_local;
        }
        else
        {
            return angle_const_cost;
        }
    }

    void belief_updateNode::updateGoalPrediction()
    {
        if (cmd_receive && path_receive)
        {
            float V_translation[goal_list.poses.size()], Q_translation[goal_list.poses.size()], Pi_translation[goal_list.poses.size()],
                V_rotation[goal_list.poses.size()], Q_rotation[goal_list.poses.size()], Pi_rotation[goal_list.poses.size()];
            std::vector<float> softmax_goal;
            float sum_belief_goal = 0, sum_translation = 0, sum_rotation = 0;
            for (int i = 0; i < goal_list.poses.size(); i++)
            {
                // ROS_INFO_STREAM("Goal " << i + 1);
                V_translation[i] = belief_updateNode::calTranslationStateValue(goal_list.poses[i]);
                Q_translation[i] = belief_updateNode::calTranslationStateActionValue(v_cmd, w_cmd, goal_list.poses[i]);
                V_rotation[i] = belief_updateNode::calRotationStateValue(goal_list.poses[i]);
                Q_rotation[i] = belief_updateNode::calRotationStateActionValue(v_cmd, w_cmd, goal_list.poses[i]);
                // ROS_INFO_STREAM("Goal " << i + 1 << ", V value " << V_g[i] << ", Q value " << Q_gu[i] << ", V+Q " << V_g[i] + Q_gu[i]);
                sum_translation += V_translation[i] + Q_translation[i];
                sum_rotation += V_rotation[i] + Q_rotation[i];
                // ROS_INFO_STREAM("Goal " << i + 1 << ", PI Value " << Pi_gu[i]);
            }
            // std::transform(belief_goal.begin(), belief_goal.end(), std::bind(std::multiplies<float>(), std::placeholders::_1, 1 / sum_belief_goal));
            for (int i = 0; i < belief_goal.size(); i++)
            {
                Pi_translation[i] = exp(-(V_translation[i] + Q_translation[i]) / sum_translation);
                Pi_rotation[i] = exp(-(V_rotation[i] + Q_rotation[i]) / sum_rotation);
                // ROS_INFO_STREAM("Goal " << i + 1 << ", Translation " << Pi_translation[i] << ", Rotation " << Pi_rotation[i]);
                // ROS_INFO_STREAM("Goal " << i + 1 << ", Scale " << (Pi_translation[i] * weight_dist + Pi_rotation[i] * weight_angle));
                softmax_goal.push_back((Pi_translation[i] * weight_dist + Pi_rotation[i] * weight_angle) * belief_goal[i]);
            }
            softmax_goal = belief_updateNode::normalize(softmax_goal);
            belief_goal = belief_updateNode::softMax(softmax_goal);
        }
    }
    
    float belief_updateNode::calDistance(float x1, float y1, float x2, float y2)
    {
        return sqrt(powf((x1 - x2), 2) + powf((y1 - y2), 2));
    }
    
    std::vector<float> belief_updateNode::softMax(std::vector<float> belief_vector)
    {
        float max_value = *std::max_element(belief_vector.begin(), belief_vector.end());
        float sum_value = 0;
        for (int i = 0; i < belief_vector.size(); i++)
        {
            sum_value += exp((belief_vector[i] - max_value) * temp_softmax);
        }
        for (int i = 0; i < belief_vector.size(); i++)
        {
            belief_vector[i] = exp((belief_vector[i] - max_value) * temp_softmax) / sum_value;
        }
        return belief_vector;
    }

    std::vector<float> belief_updateNode::clipBelief(std::vector<float> belief_vector)
    {
        for (int i = 0; i < belief_vector.size(); i++)
        {
            if (belief_vector[i] < lower_bound)
            {
                belief_vector[i] = lower_bound;
                continue;
            }
            else if (belief_vector[i] > upper_bound)
            {
                belief_vector[i] = upper_bound;
            }
        }
        return belief_updateNode::normalize(belief_vector);
    }

    std::vector<float> belief_updateNode::normalize(std::vector<float> belief_vector)
    {
        float sum_value = 0;
        for (int i = 0; i < belief_vector.size(); i++)
        {
            sum_value += belief_vector[i];
        }
        for (int i = 0; i < belief_vector.size(); i++)
        {
            belief_vector[i] = belief_vector[i] / sum_value;
        }
        return belief_vector;
    }
   }
