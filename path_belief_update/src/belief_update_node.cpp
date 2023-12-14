#include "path_belief_update/belief_update_node.h"

namespace belief_update
{
    // Constructor
    belief_updateNode::belief_updateNode(ros::NodeHandle &node_handle)
    :   node_handle_(node_handle), tf_listener(tf_buffer)
    {
        if (!readParameters())
        {
            ROS_ERROR("Could not load parameters.");
            ros::requestShutdown();
        }
        // Subscribers & Publishers
        odom_subscriber_ = node_handle_.subscribe("/odom", 1, &belief_updateNode::odomCallback, this);
        path_subscriber_ = node_handle_.subscribe("/all_paths", 1, &belief_updateNode::pathCallback, this);
        status_subscriber_ = node_handle_.subscribe("/move_base/status", 1, &belief_updateNode::statusCallback, this);
        global_subscriber_ = node_handle_.subscribe("/move_base/goal", 1, &belief_updateNode::globalCallback, this);
        if (input_datatype == "point")
        {
            input_subscriber_ = node_handle_.subscribe("/joystick_calib", 1, &belief_updateNode::pointCallback, this);
        }
        else if (input_datatype == "twist")
        {
            input_subscriber_ = node_handle_.subscribe("/joy_vel", 1, &belief_updateNode::twistCallback, this);
        }

        // vel_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/belief_update/cmd_vel", 1);
        goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseArray>("/goal_distribution", 1, true);
        path_publisher_ = node_handle_.advertise<std_msgs::UInt32>("/preferred_path_ind", 1, true);
        local_publiser_ = node_handle_.advertise<geometry_msgs::Pose>("/belief_update/global_goal", 1);
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
        if (!node_handle_.getParam("path_frame_id", path_frame_id))
            ROS_WARN_STREAM("Parameter path_frame_id not set. Using default setting: " << path_frame_id);
        if (!node_handle_.getParam("base_frame_id", base_frame_id))
            ROS_WARN_STREAM("Parameter base_frame_id not set. Using default setting: " << base_frame_id);
        if (!node_handle_.getParam("odom_frame_id", odom_frame_id))
            ROS_WARN_STREAM("Parameter odom_frame_id not set. Using default setting: " << odom_frame_id);
        if (!node_handle_.getParam("rate_factor", rate_factor))
            ROS_WARN_STREAM("Parameter rate_factor not set. Using default setting: " << rate_factor);
        if (!node_handle_.getParam("discount_factor", discount_factor))
            ROS_WARN_STREAM("Parameter discount_factor not set. Using default setting: " << discount_factor);
        if (!node_handle_.getParam("angle_threshold", angle_threshold))
            ROS_WARN_STREAM("Parameter angle_threshold not set. Using default setting: " << angle_threshold);
        if (!node_handle_.getParam("angle_const_cost", angle_const_cost))
            ROS_WARN_STREAM("Parameter angle_const_cost not set. Using default setting: " << angle_const_cost);
        if (!node_handle_.getParam("tf_buffer_timeout", tf_buffer_timeout))
            ROS_WARN_STREAM("Parameter tf_buffer_timeout not set. Using default setting: " << tf_buffer_timeout);
        if (!node_handle_.getParam("temp_softmax", temp_softmax))
            ROS_WARN_STREAM("Parameter temp_softmax not set. Using default setting: " << temp_softmax);
        if (!node_handle_.getParam("upper_bound", upper_bound))
            ROS_WARN_STREAM("Parameter upper_bound not set. Using default setting: " << upper_bound);
        if (!node_handle_.getParam("lower_bound", lower_bound))
            ROS_WARN_STREAM("Parameter lower_bound not set. Using default setting: " << lower_bound);
        if (!node_handle_.getParam("sample_time", sample_time))
            ROS_WARN_STREAM("Parameter sample_time not set. Using default setting: " << sample_time);
        if (!node_handle_.getParam("waypoint_increment", waypoint_increment))
            ROS_WARN_STREAM("Parameter waypoint_increment not set. Using default setting: " << waypoint_increment);
        if (!node_handle_.getParam("input_datatype", input_datatype))
            ROS_WARN_STREAM("Parameter input_datatype not set. Using default setting: " << input_datatype);
        if (!node_handle_.getParam("x_max", x_max))
            ROS_WARN_STREAM("Parameter x_max not set. Using default setting: " << x_max);
        if (!node_handle_.getParam("y_max", y_max))
            ROS_WARN_STREAM("Parameter y_max not set. Using default setting: " << y_max);
        if (input_datatype == "point")
        {
            ROS_INFO_STREAM("Input datatype is \"point\".");
        }
        else if (input_datatype == "twist")
        {
            ROS_INFO_STREAM("Input datatype is \"twist\".");
        }
        else
        {
            ROS_ERROR_STREAM("The selected input_datatype is not supported, please refer to the options in config file!");
            return false;
        }
        ROS_INFO_STREAM("Complete loading parameters.");
        return true;
    }

    void belief_updateNode::odomCallback(const nav_msgs::Odometry &msg_odom)
    {
        odom_frame_id = msg_odom.header.frame_id;
        v_agent = msg_odom.twist.twist.linear.x;
        w_agent = msg_odom.twist.twist.angular.z;
        waypoint_dist = fabs(v_agent) * sample_time + waypoint_increment;
        // v_agent = round(v_agent * 10) / 10;
        // w_agent = round(w_agent * 10) / 10;
        odom_receive = true;
        // ROS_INFO_STREAM("Odom received.");
    }

    void belief_updateNode::twistCallback(const geometry_msgs::Twist &msg_twist)
    {
        x_cmd = msg_twist.linear.x;
        y_cmd = msg_twist.angular.z;
        x_cmd /= x_max;
        y_cmd /= y_max;
        x_cmd = round(x_cmd * 100) / 100;
        y_cmd = round(y_cmd * 100) / 100;
        if (x_cmd > 0 || (x_cmd == 0 && y_cmd != 0))
        {
            action_update = true;
            
        }
        else
        {
            noaction_update = true;
        }
        input_receive = true;
    }
    
    void belief_updateNode::pointCallback(const geometry_msgs::Point &msg_point)
    {
        x_cmd = msg_point.x;
        y_cmd = msg_point.y;
        x_cmd /= x_max;
        y_cmd /= y_max;
        x_cmd = round(x_cmd * 100) / 100;
        y_cmd = round(y_cmd * 100) / 100;
        if (x_cmd > 0 || (x_cmd == 0 && y_cmd != 0))
        {
            action_update = true;
        }
        else
        {
            noaction_update = true;
        }
        input_receive = true;
    }

    void belief_updateNode::statusCallback(const actionlib_msgs::GoalStatusArray &msg_status)
    {
        if (msg_status.status_list.size() != 0)
        {
            if (msg_status.status_list[0].status == 3)
            {
                goal_reached = true;
            }
            else
            {
                goal_reached = false;
            }
            status_receive = true;
        }
    }

    void belief_updateNode::globalCallback(const move_base_msgs::MoveBaseActionGoal &msg_global)
    {
        global_goal = msg_global.goal.target_pose;
        global_receive = true;
    }

    void belief_updateNode::pathCallback(const voronoi_msgs_and_types::PathList &msg_path)
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
            // ROS_INFO_STREAM("Stamp: " << path_list.paths[0].header.stamp);
            // for (int i = 0; i < path_list.paths.size(); ++i)
            // {
            //     ROS_INFO_STREAM("Path " << i + 1 << ", seq = " << path_list.paths[i].header.seq);
            // }
            path_frame_id = path_list.paths[0].header.frame_id;
            if (belief_goal.size() == 0) // initialization
            {
                if (!goal_reached)
                {
                    ROS_INFO_STREAM("Belief Space Initializing.");
                    pre_path_list = path_list;
                    belief_goal.resize(path_list.paths.size(), 1 / static_cast<float>(path_list.paths.size()));
                    // std::cout << "path size " <<  path_list.paths.size() << std::endl;
                    // for (int i = 0; i < path_list.paths.size(); i++)
                    // {
                    //     // belief_goal[i] =  1 / static_cast<float>(belief_goal.size());
                    //     std::cout << "belief_goal " << i << ": " <<  belief_goal[i] << std::endl;
                    // }
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
                if (action_update)
                {
                    belief_goal = belief_updateNode::softMax(belief_goal);
                    belief_goal = belief_updateNode::clipBelief(belief_goal);
                }
            }
            else // no new paths added, only replacement
            {
                // ROS_INFO_STREAM("Amount of paths not changed.");
                int num_changed_goal = 0;
                float sum_changed_goal = 0;
                bool path_replaced = false;
                for (int i = 0; i < path_list.paths.size(); i++)
                {
                    if (pre_path_list.paths[i].header.seq == path_list.paths[i].header.seq)
                    {
                        continue;
                    }
                    num_changed_goal++;
                    sum_changed_goal += belief_goal[i];
                    belief_goal[i] = 0;
                    path_replaced = true;
                    ROS_INFO_STREAM( num_changed_goal << " path(s) has changed.");
                }
                if (path_replaced)
                {
                    for (int i = 0; i < path_list.paths.size(); i++)
                    {
                        if (pre_path_list.paths[i].header.seq == path_list.paths[i].header.seq)
                        {
                            belief_goal[i] += sum_changed_goal / (belief_goal.size() - num_changed_goal);
                        }
                    }
                    if (action_update)
                    {
                        belief_goal = belief_updateNode::softMax(belief_goal);
                        belief_goal = belief_updateNode::clipBelief(belief_goal);
                    }
                }
                if (num_changed_goal == belief_goal.size())
                {
                    ROS_INFO_STREAM("All paths are changed, re-initializing...");
                    belief_goal.clear();
                    belief_goal.resize(path_list.paths.size(), 1 / static_cast<float>(path_list.paths.size()));
                }
                pre_path_list = path_list;
                // ROS_INFO_STREAM("path_list.paths.size() " << path_list.paths.size());
                // ROS_INFO_STREAM("Before Softmax:");
                // for (int i = 0; i < belief_goal.size(); i++)
                // {
                //     ROS_INFO_STREAM("Belief Goal " << i + 1 << ": " << belief_goal[i]);
                // }
                // ROS_INFO_STREAM("After Softmax:");
                // for (int i = 0; i < belief_goal.size(); i++)
                // {
                //     ROS_INFO_STREAM("Belief Goal " << i + 1 << ": " << belief_goal[i]);
                // }
            }
            path_receive = true;

            //Recalculate goal and publish
            // if(goal_index >= 0 && goal_index < path_list.paths.size())
            // {
            //     agent_pose = getGlobalAgentPose().pose;
            //     auto temp_goal = findGoal(agent_pose, path_list.paths[goal_index]);

            //     geometry_msgs::TransformStamped map2localTransform;
            //     try
            //     {
            //         map2localTransform = tf_buffer.lookupTransform(base_frame_id, path_frame_id, ros::Time(0), ros::Duration(0.10));
            //         tf2::doTransform<geometry_msgs::Pose>(temp_goal, temp_goal, map2localTransform);
            //         goal_publisher_.publish(temp_goal);
            //     }
            //     catch (tf2::TransformException &Exception) 
            //     {
            //         ROS_ERROR_STREAM(Exception.what());
            //     }
            // }
            if (!goal_reached)
            {
                belief_updateNode::generateGoal();
                // belief_updateNode::updateGoalPrediction();
            }
        }
    }

    void belief_updateNode::noactionTimerCallback(const ros::TimerEvent&)
    {
        if (noaction_update && odom_receive)
        {
            if (!goal_reached)
            {
                belief_updateNode::generateGoal();
                // belief_updateNode::updateGoalPrediction();
            }
            belief_updateNode::noactionPublishResults();
            noaction_update = false;
        }
    }

    void belief_updateNode::actionTimerCallback(const ros::TimerEvent&)
    {
        if (action_update && odom_receive)
        {
            if (!goal_reached)
            {
                belief_updateNode::generateGoal();
                belief_updateNode::updateGoalPrediction();
                for (int i = 0; i < goal_list.poses.size(); i++)
                {
                    // store the probabilities of goals in z coordinate
                    goal_list.poses[i].position.z = belief_goal[i];
                }
            }
            belief_updateNode::actionPublishResults();
            action_update = false;
        }
    }

    geometry_msgs::PoseStamped belief_updateNode::getGlobalAgentPose()
    {
        geometry_msgs::PoseStamped agent_pose_temp;
        agent_pose_temp.header.frame_id = base_frame_id;
        agent_pose_temp.pose.orientation.w = 1.0;

        try{
            auto tf = tf_buffer.lookupTransform(path_frame_id, base_frame_id, ros::Time(0), ros::Duration(0.15));
            tf2::doTransform<geometry_msgs::PoseStamped>(agent_pose_temp, agent_pose_temp, tf);
        }
        catch(tf2::TransformException &exception){
            ROS_ERROR("%s", exception.what());
        }

        return agent_pose_temp;
    }

    void belief_updateNode::generateGoal()
    {
        if (path_receive)
        {
            // geometry_msgs::Point point_marker;
            goal_list.poses.clear();
            goal_list.poses.resize(path_list.paths.size());
            agent_pose = getGlobalAgentPose().pose;
            for (int i = 0; i < path_list.paths.size(); i++)
            {
                goal_list.poses[i] = belief_updateNode::findGoal(agent_pose, path_list.paths[i]);
                // point_marker.x = goal_list.poses[i].position.x;
                // point_marker.y = goal_list.poses[i].position.y;
                // point_marker.z = 0;
                // goal_marker.points.push_back(point_marker);
            }

            // transform from map to base_link
            geometry_msgs::TransformStamped map2localTransform;
            try
            {
                map2localTransform = tf_buffer.lookupTransform(base_frame_id, path_frame_id, ros::Time(0), ros::Duration(tf_buffer_timeout));
            }
            catch (tf2::TransformException &Exception) 
            {
                ROS_ERROR_STREAM(Exception.what());
            }
            tf2::doTransform<geometry_msgs::PoseStamped>(global_goal, global_goal_local, map2localTransform);
            for (int i = 0; i < goal_list.poses.size(); i++)
            {
                tf2::doTransform<geometry_msgs::Pose>(goal_list.poses[i], goal_list.poses[i], map2localTransform);
                // store the probabilities of goals in z coordinate
                goal_list.poses[i].position.z = belief_goal[i];
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

    void belief_updateNode::actionPublishResults()
    {
        // ROS_INFO_STREAM("path_receive " << path_receive << ", goal_reached " << goal_reached << ", global_receive " << global_receive);
        if (path_receive && !goal_reached && global_receive)
        {
            // ROS_INFO_STREAM("Update after user action:");
            // for (int i = 0; i < belief_goal.size(); i++)
            // {
            //     ROS_INFO_STREAM("Belief Goal " << i + 1 << ": " << belief_goal[i]);
            // }
            local_publiser_.publish(global_goal_local.pose);
            std::vector<float>::iterator itMax = std::max_element(belief_goal.begin(), belief_goal.end());
            goal_index = std::distance(belief_goal.begin(), itMax);
            goal_publisher_.publish(goal_list);
            std_msgs::UInt32 path_index;
            path_index.data = path_list.paths[goal_index].header.seq;
            path_publisher_.publish(path_index);
            
        }
        else
        {
            goal_publisher_.publish(geometry_msgs::PoseArray());
            goal_index = -1;
        }
        input_receive = false;
        path_receive = false;
        odom_receive = false;
        status_receive = false;
        goal_list.poses.clear();
        if (goal_reached)
        {
            belief_goal.clear();
        }
    }

    void belief_updateNode::noactionPublishResults()
    {
        // ROS_INFO_STREAM("path_receive " << path_receive << ", goal_reached " << goal_reached << ", global_receive " << global_receive);
        if (path_receive && !goal_reached && global_receive)
        {
            // ROS_INFO_STREAM("Update after no user action:");
            // for (int i = 0; i < belief_goal.size(); i++)
            // {
            //     ROS_INFO_STREAM("Belief Goal " << i + 1 << ": " << belief_goal[i]);
            // }
            local_publiser_.publish(global_goal_local.pose);
            std::vector<float>::iterator itMax = std::max_element(belief_goal.begin(), belief_goal.end());
            goal_index = std::distance(belief_goal.begin(), itMax);
            goal_publisher_.publish(goal_list);
            std_msgs::UInt32 path_index;
            path_index.data = path_list.paths[goal_index].header.seq;
            path_publisher_.publish(path_index);
        }
        else
        {
            goal_publisher_.publish(geometry_msgs::PoseArray());
            goal_index = -1;
        }
        input_receive = false;
        path_receive = false;
        odom_receive = false;
        status_receive = false;
        goal_list.poses.clear();
        if (goal_reached)
        {
            belief_goal.clear();
        }
    }

    float belief_updateNode::calRotationValue(float x_input, float y_input, geometry_msgs::Pose goal_pose)
    {
        tf2::Vector3 agent2goal(goal_pose.position.x, goal_pose.position.y, 0);
        tf2::Vector3 agent_heading(1, 0, 0);
        tf2::Vector3 point2goal(x_input, y_input, 0);
        float V_current = agent_heading.angle(agent2goal);
        float V_next;
        if (x_input == 0 && y_input == 0)
        {
            V_next = PI;
        }
        else
        {
            V_next = agent2goal.angle(point2goal);
        }
        float C_action = belief_updateNode::calRotationActionCost(V_current);
        // ROS_INFO_STREAM("x_input " << x_input << ", y_input " << y_input);
        // ROS_INFO_STREAM("V_current " << V_current << ", C_action " << C_action << ", V_next " << V_next);
        return discount_factor * V_next + C_action; //+ V_current;
    }

    float belief_updateNode::calRotationActionCost(float angle_next)
    {
        float angle_threshold_local = M_PI / 2;
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
        if (input_receive && path_receive)
        {
            float Q_rotation[goal_list.poses.size()], Pi_rotation[goal_list.poses.size()];
            std::vector<float> softmax_goal;
            float sum_rotation = 0;
            for (int i = 0; i < goal_list.poses.size(); i++)
            {
                // ROS_INFO_STREAM("Goal " << i + 1);
                Q_rotation[i] = belief_updateNode::calRotationValue(x_cmd, y_cmd, goal_list.poses[i]);
                // ROS_INFO_STREAM("Goal " << i + 1 << ", V value " << V_g[i] << ", Q value " << Q_gu[i] << ", V+Q " << V_g[i] + Q_gu[i]);
                // sum_rotation += Q_rotation[i];
                Pi_rotation[i] = exp(- rate_factor * Q_rotation[i]);
                softmax_goal.push_back(Pi_rotation[i] * belief_goal[i]);
                // ROS_INFO_STREAM("Goal " << i + 1 << ", Q Value " << Q_rotation[i]);
            }
            // std::transform(belief_goal.begin(), belief_goal.end(), std::bind(std::multiplies<float>(), std::placeholders::_1, 1 / sum_belief_goal));
            // Pi_rotation =  exp(- ModelParams::exp_temperature * Q_rotation);
            // for (int i = 0; i < belief_goal.size(); i++)
            // {
            //     // Pi_rotation[i] = exp(- Q_rotation[i] / sum_rotation);
            //     Pi_rotation[i] = powf((1 - Q_rotation[i] / sum_rotation), rate_factor);
            //     // ROS_INFO_STREAM("Goal " << i + 1 << ", Pi value " << Pi_rotation[i]);
            //     softmax_goal.push_back(Pi_rotation[i] * belief_goal[i]);
            // }
            // ROS_INFO_STREAM("After update: ");
            // for (int i = 0; i < belief_goal.size(); i++)
            // {
            //     ROS_INFO_STREAM("Belief Goal " << i + 1 << ": " << softmax_goal[i]);
            // }
            softmax_goal = belief_updateNode::normalize(softmax_goal);
            // belief_goal = belief_updateNode::softMax(softmax_goal);
            belief_goal = belief_updateNode::clipBelief(softmax_goal);
            // ROS_INFO_STREAM("After processing: ");
            // for (int i = 0; i < belief_goal.size(); i++)
            // {
            //     ROS_INFO_STREAM("Belief Goal " << i + 1 << ": " << belief_goal[i]);
            // }
        }
    }
    
    float belief_updateNode::calDistance(float x1, float y1, float x2, float y2)
    {
        return sqrt(powf((x1 - x2), 2) + powf((y1 - y2), 2));
    }
    
    std::vector<float> belief_updateNode::softMax(std::vector<float> belief_vector)
    {
        int num_paths = path_list.paths.size();
        float max_value = *std::max_element(belief_vector.begin(), belief_vector.end());
        float sum_value = 0;
        for (int i = 0; i < belief_vector.size(); i++)
        {
            sum_value += exp((belief_vector[i] - max_value) * static_cast<float> (num_paths - 1) * temp_softmax);
        }
        for (int i = 0; i < belief_vector.size(); i++)
        {
            belief_vector[i] = exp((belief_vector[i] - max_value) * static_cast<float> (num_paths - 1) * temp_softmax) / sum_value;
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
