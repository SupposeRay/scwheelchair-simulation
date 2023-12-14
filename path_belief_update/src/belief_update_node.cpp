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
        if (input_datatype == "point")
        {
            input_subscriber_ = node_handle_.subscribe("/joystick", 1, &belief_updateNode::pointCallback, this);
        }
        else if (input_datatype == "twist")
        {
            input_subscriber_ = node_handle_.subscribe("/inputer_converter/cmd_vel", 1, &belief_updateNode::twistCallback, this);
        }

        goal_publisher_ = node_handle_.advertise<path_belief_update::WaypointDistribution>("/waypoint_distribution", 1, true);
        path_publisher_ = node_handle_.advertise<std_msgs::UInt32>("/preferred_path_ind", 1, true);
        belief_timer_ = node_handle_.createTimer(ros::Duration(update_interval), &belief_updateNode::beliefTimerCallback, this);
        publish_timer_ = node_handle_.createTimer(ros::Duration(publish_interval), &belief_updateNode::publishTimerCallback, this);
    }
    // Destructor
    belief_updateNode::~belief_updateNode()
    {}
    //Public Member Functions

    //Private Member Functions
    bool belief_updateNode::readParameters()
    {
        ROS_INFO_STREAM("Loading parameters.....");
        if (!node_handle_.getParam("update_interval", update_interval))
            ROS_WARN_STREAM("Parameter update_interval not set. Using default setting: " << update_interval);
        if (!node_handle_.getParam("publish_interval", publish_interval))
            ROS_WARN_STREAM("Parameter publish_interval not set. Using default setting: " << publish_interval);
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
        // waypoint_dist = fabs(v_agent) * 1 + waypoint_increment;
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
        x_cmd = round(x_cmd * 10) / 10;
        y_cmd = round(y_cmd * 10) / 10;
        if (x_cmd > 0 || (x_cmd == 0 && y_cmd != 0))
        {
            action_update = true;
        }
        else
        {
            action_update = false;
        }
        input_receive = true;
    }
    
    void belief_updateNode::pointCallback(const geometry_msgs::Point &msg_point)
    {
        x_cmd = msg_point.x;
        y_cmd = msg_point.y;
        x_cmd /= x_max;
        y_cmd /= y_max;
        x_cmd = round(x_cmd * 10) / 10;
        y_cmd = round(y_cmd * 10) / 10;
        if (x_cmd > 0 || (x_cmd == 0 && y_cmd != 0))
        {
            action_update = true;
        }
        else
        {
            action_update = false;
        }
        input_receive = true;
    }

    void belief_updateNode::pathCallback(const voronoi_msgs_and_types::PathList &msg_path)
    {
        path_list = msg_path;
        // ROS_INFO_STREAM("Pathlist received:");
        // for (int i = 0; i < path_list.paths.size(); i++)
        // {
        //     ROS_INFO_STREAM("Path " << i << ", seq " << path_list.paths[i].header.seq);
        // }
        // ROS_INFO_STREAM("Number of Paths: " << path_list.paths.size());
        // check if the paths exist or have changed
        if (path_list.paths.size() == 0)
        {
            ROS_WARN_STREAM("Path candidates not received!");
            path_receive = false;
            return;
        }
        else
        {
            belief_updateNode::pathToLocal(path_list);
            // Generate the waypoints when the paths are received
            belief_updateNode::generateGoal();
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
            else
            {
                std::vector<int> index_prepath_inherit, index_newpath_inherit;
                for (int i = 0; i < pre_path_list.paths.size(); i++)
                {
                    std::vector<nav_msgs::Path>::iterator it_path = 
                    std::find_if(path_list.paths.begin(), path_list.paths.end(), belief_update::seq_finder(pre_path_list.paths[i].header.seq));
                    if (it_path != path_list.paths.end())
                    {
                        //this path is inherited
                        index_prepath_inherit.push_back(i);
                        index_newpath_inherit.push_back(it_path - path_list.paths.begin());
                    }
                }
                belief_goal = belief_updateNode::redistributeProb(pre_path_list, path_list, index_prepath_inherit, index_newpath_inherit, belief_goal);
            }

            pre_path_list = path_list;
            path_receive = true;
        }
    }

    void belief_updateNode::beliefTimerCallback(const ros::TimerEvent&)
    {
        if (action_update && odom_receive)
        {
            // belief_updateNode::generateGoal();
            belief_updateNode::updateGoalPrediction();
            // action_update = false;
        }
    }

    void belief_updateNode::publishTimerCallback(const ros::TimerEvent&)
    {
        belief_updateNode::PublishResults();
    }

    /*
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
    }*/

    void belief_updateNode::pathToLocal(voronoi_msgs_and_types::PathList &msg_path)
    {
        path_frame_id = path_list.paths[0].header.frame_id;
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

        for (int i = 0; i < msg_path.paths.size(); i++)
        {
            for (int j = 0; j < msg_path.paths[i].poses.size(); j++)
            {
                tf2::doTransform<geometry_msgs::PoseStamped>(msg_path.paths[i].poses[j], msg_path.paths[i].poses[j], map2localTransform);
            }
        }
    }

    std::vector<float> belief_updateNode::redistributeProb(voronoi_msgs_and_types::PathList old_path, voronoi_msgs_and_types::PathList new_path, std::vector<int> old_index, std::vector<int> new_index, std::vector<float> prob_path)
    {
        std::vector<float> new_prob(new_path.paths.size(), 0.0);    // the new probability distribution
        std::vector<int> removed_index, added_index;    // the removed paths of previous step and added paths of next step
        std::vector<tf2::Vector3> old_directions, new_directions;   // directions of different paths in previous and next steps
        tf2::Vector3 temp_vector;
        // get directions of all paths in previous step, using vectors pointing from base_link to the 5th point along each path
        int n = 0;
        for(int i = 0; i < old_path.paths.size(); i++)
        {
            for(int j = 0; j < old_path.paths[i].poses.size(); j++)
            {
                if (j > 2 || j == old_path.paths[i].poses.size() - 1)
                {
                    temp_vector.setX(old_path.paths[i].poses[j].pose.position.x);
                    temp_vector.setY(old_path.paths[i].poses[j].pose.position.y);
                    temp_vector.setZ(0);
                    old_directions.push_back(temp_vector);
                    break;
                }
            }
            // check if this path is removed
            if (std::find(old_index.begin(), old_index.end(), i) == old_index.end())
            {
                // path is removed
                removed_index.push_back(i);
            }
            else
            {
                // path is not removed, inherit the probability
                new_prob[new_index[n]] = prob_path[i];
                n++;
            }
        }
        // get directions of all paths in next step, using vectors pointing from base_link to the 5th point along each path
        for(int i = 0; i < new_path.paths.size(); i++)
        {
            for(int j = 0; j < new_path.paths[i].poses.size(); j++)
            {
                if (j > 2 || j == new_path.paths[i].poses.size() - 1)
                {
                    temp_vector.setX(new_path.paths[i].poses[j].pose.position.x);
                    temp_vector.setY(new_path.paths[i].poses[j].pose.position.y);
                    temp_vector.setZ(0);
                    new_directions.push_back(temp_vector);
                    break;
                }
            }
            // check if this path is newly added
            if (std::find(new_index.begin(), new_index.end(), i) == new_index.end())
            {
                added_index.push_back(i);
            }
        }
        // ROS_INFO_STREAM("Before merging:");
        // for (int i = 0; i < prob_path.size(); i++)
        // {
        //     ROS_INFO_STREAM("Belief " << i << ": " << prob_path[i] << ", seq " << old_path.paths[i].header.seq);
        // }
        std::vector<float> angle_diff;
        float angle_sum = 0;
        // distribute the probabilities of removed paths to new paths based on the angle difference
        for (int i = 0; i < removed_index.size(); i++)
        {
            for (int j = 0; j < new_path.paths.size(); j++)
            {
                float angle_vec_diff = PI - old_directions[removed_index[i]].angle(new_directions[j]);
                angle_sum += angle_vec_diff;
                angle_diff.push_back(angle_vec_diff);
            }
            for (int j = 0; j < new_path.paths.size(); j++)
            {
                new_prob[j] += prob_path[removed_index[i]] * angle_diff[j] / angle_sum;
            }
            angle_diff.clear();
            angle_sum = 0;
        }
        // distribute the probabilities of removed paths to other remained paths based on the angle difference
        // for (int i = 0; i < removed_index.size(); i++)
        // {
        //     for (int j = 0; j < old_index.size(); j++)
        //     {
        //         float angle_vec_diff = PI - old_directions[removed_index[i]].angle(old_directions[old_index[j]]);
        //         angle_sum += angle_vec_diff;
        //         angle_diff.push_back(angle_vec_diff);
        //     }
        //     for (int j = 0; j < old_index.size(); j++)
        //     {
        //         new_prob[new_index[j]] += prob_path[removed_index[i]] * angle_diff[j] / angle_sum;
        //     }
        //     angle_diff.clear();
        //     angle_sum = 0;
        // }
        // distribute the probabilities of removed paths to other remained paths evenly
        // for (int i = 0; i < removed_index.size(); i++)
        // {
        //     for (int j = 0; j < old_index.size(); j++)
        //     {
        //         new_prob[new_index[j]] += prob_path[removed_index[i]] / old_index.size();
        //     }
        // }
        // ROS_INFO_STREAM("After merging:");
        // for (int i = 0; i < new_index.size(); i++)
        // {
        //     ROS_INFO_STREAM("Belief " << i << ": " << new_prob[new_index[i]] << ", seq " << new_path.paths[new_index[i]].header.seq);
        // }

        // add probabilities to newly added paths based on the angle difference between other remained paths
        for (int i = 0; i < added_index.size(); i++)
        {
            for (int j = 0; j < new_index.size(); j++)
            {
                angle_diff.push_back(new_directions[added_index[i]].angle(new_directions[new_index[j]]));
            }
            // find the nearest path
            std::vector<float>::iterator it = std::min_element(angle_diff.begin(), angle_diff.end());
            // assign a probability to the new path based on the probability of its nearest path
            new_prob[added_index[i]] += new_prob[new_index[it - angle_diff.begin()]] * (PI - *it) / PI;
            angle_diff.clear();
        }
        // ROS_INFO_STREAM("After inserting:");
        // for (int i = 0; i < new_prob.size(); i++)
        // {
        //     ROS_INFO_STREAM("Belief " << i << ": " << new_prob[i] << ", seq " << new_path.paths[i].header.seq);
        // }
        if (added_index.size() != 0)
        {
            // new_prob = belief_updateNode::softMax(new_prob);
            new_prob = belief_updateNode::clipBelief(new_prob);
        }
        // ROS_INFO_STREAM("After clipping:");
        // for (int i = 0; i < new_prob.size(); i++)
        // {
        //     ROS_INFO_STREAM("Belief " << i << ": " << new_prob[i] << ", seq " << new_path.paths[i].header.seq);
        // }
        return new_prob;
    }

    void belief_updateNode::generateGoal()
    {
        // geometry_msgs::Point point_marker;
        goal_list.poses.resize(path_list.paths.size());
        for (int i = 0; i < path_list.paths.size(); i++)
        {
            goal_list.poses[i] = belief_updateNode::findGoal(path_list.paths[i]);
            // point_marker.x = goal_list.poses[i].position.x;
            // point_marker.y = goal_list.poses[i].position.y;
            // point_marker.z = 0;
            // goal_marker.points.push_back(point_marker);
        }
        goal_list.header.frame_id = base_frame_id;
    }

    geometry_msgs::Pose belief_updateNode::findGoal(const nav_msgs::Path &msg_path)
    {
        geometry_msgs::Pose generated_goal;
        
        tf2::Vector3 goal_direction;
        tf2::Quaternion goal_quat;

        float acmlt_dist = belief_updateNode::calDistance(0, 0, msg_path.poses[0].pose.position.x, msg_path.poses[0].pose.position.y);
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

    void belief_updateNode::PublishResults()
    {
        if (path_receive)
        {
            // ROS_INFO_STREAM("After update:");
            // for (int i = 0; i < belief_goal.size(); i++)
            // {
            //     ROS_INFO_STREAM("Belief Goal " << i + 1 << ": " << belief_goal[i]);
            // }
            waypoint_belief.distribution = belief_goal;
            waypoint_belief.waypoints = goal_list;
            goal_publisher_.publish(waypoint_belief);
        }
        else
        {
            goal_publisher_.publish(path_belief_update::WaypointDistribution());
        }
        input_receive = false;
        path_receive = false;
        odom_receive = false;
        goal_list.poses.clear();
        // if (goal_reached)
        // {
        //     belief_goal.clear();
        // }
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
        return discount_factor * V_next + C_action + V_current;
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
        if (input_receive && path_receive)
        {
            // ROS_INFO_STREAM("x_cmd " << x_cmd << ", y_cmd " << y_cmd);
            // ROS_INFO_STREAM("Before update:");
            // for (int i = 0; i < belief_goal.size(); i++)
            // {
            //     ROS_INFO_STREAM("Belief Goal " << i + 1 << ": " << belief_goal[i]);
            // }
            float Q_rotation[goal_list.poses.size()], Pi_rotation[goal_list.poses.size()];
            std::vector<float> softmax_goal;
            float sum_rotation = 0;
            for (int i = 0; i < goal_list.poses.size(); i++)
            {
                // ROS_INFO_STREAM("Goal " << i + 1);
                Q_rotation[i] = belief_updateNode::calRotationValue(x_cmd, y_cmd, goal_list.poses[i]);
                // ROS_INFO_STREAM("Goal " << i + 1 << ", V value " << V_g[i] << ", Q value " << Q_gu[i] << ", V+Q " << V_g[i] + Q_gu[i]);
                sum_rotation += Q_rotation[i];
                // ROS_INFO_STREAM("Goal " << i + 1 << ", Q Value " << Q_rotation[i]);
            }
            // std::transform(belief_goal.begin(), belief_goal.end(), std::bind(std::multiplies<float>(), std::placeholders::_1, 1 / sum_belief_goal));
            for (int i = 0; i < belief_goal.size(); i++)
            {
                Pi_rotation[i] = exp(- Q_rotation[i] / sum_rotation);
                // Pi_rotation[i] = powf((1 - Q_rotation[i] / sum_rotation), rate_factor);
                // ROS_INFO_STREAM("Goal " << i + 1 << ", Pi value " << Pi_rotation[i]);
                softmax_goal.push_back(Pi_rotation[i] * belief_goal[i]);
            }
            // softmax_goal = belief_updateNode::normalize(softmax_goal);
            // belief_goal = belief_updateNode::softMax(softmax_goal);
            belief_goal = belief_updateNode::clipBelief(softmax_goal);
            ROS_INFO_STREAM("After update:");
            for (int i = 0; i < belief_goal.size(); i++)
            {
                ROS_INFO_STREAM("Belief Goal " << i << ": " << belief_goal[i]);
            }
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
