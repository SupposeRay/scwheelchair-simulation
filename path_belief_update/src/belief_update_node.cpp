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
        viz_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("/visualization/circle", 1);
        belief_timer_ = node_handle_.createTimer(ros::Duration(update_interval), &belief_updateNode::beliefTimerCallback, this);
        publish_timer_ = node_handle_.createTimer(ros::Duration(publish_interval), &belief_updateNode::publishTimerCallback, this);

        if (circle_mode == "direction")
        {
            ROS_INFO_STREAM("Circle Space Initializing...");
            belief_circle.resize(36, static_cast<float>(1.0/36));
            angle_circle.resize(36, 0);
            // visualization markers
            circle_line.header.stamp = ros::Time::now();
            circle_line.header.frame_id = base_frame_id;
            circle_line.ns = "visualization_circle";
            circle_line.action = visualization_msgs::Marker::ADD;
            circle_line.pose.orientation.w = 1.0;
            circle_line.type = visualization_msgs::Marker::SPHERE_LIST;
            circle_line.scale.x = 0.1;
            circle_line.scale.y = 0.1;
            circle_line.scale.z = 0.1;
            circle_line.color.g = 1.0;
            circle_line.color.a = 1.0;
            circle_line.pose.orientation.w = 1.0;
            circle_line.points.reserve(10);

            tf2::Quaternion rotation_angle;
            geometry_msgs::Point circle_point;
            geometry_msgs::Quaternion tf_quat_msg;
            geometry_msgs::TransformStamped rotation_tf;
            rotation_tf.header.frame_id = base_frame_id;
            rotation_tf.child_frame_id = base_frame_id;
            rotation_tf.transform.translation.x = 0;
            rotation_tf.transform.translation.y = 0;
            rotation_tf.transform.translation.z = 0;
            // tf2::doTransform<geometry_msgs::PoseStamped>(agent_pose_temp, agent_pose_temp, tf);
            for (int i = 0; i < angle_circle.size(); i++)
            {
                angle_circle[i] = i * M_PI / 18;

                circle_line.id = i;
                circle_line.color.g = 1.0 - powf(belief_circle[i], 0.2);
                circle_line.color.r = 1;
                // circle_line.color.g = 1.0 - powf(belief_circle[i], 0.6);
                // circle_line.color.r = powf(belief_circle[i], 0.2);
                // circle_line.color.b = powf(belief_circle[i], 0.2);
                tf2::Vector3 angle_direction;
                angle_direction.setX(1);
                angle_direction.setY(0);
                angle_direction.setZ(0);
                tf2::Matrix3x3 angle_rotation;
                angle_rotation.setRPY(0, 0, angle_circle[i]);
                angle_direction = angle_rotation * angle_direction;
                circle_global_direction.push_back(angle_direction);
                for (int j = 0; j < 10; j++)
                {
                    angle_direction.setX((j + 1) * 0.1);
                    angle_direction.setY(0);
                    angle_direction.setZ(0);
                    angle_direction = angle_rotation * angle_direction;
                    circle_point.x = angle_direction.getX();
                    circle_point.y = angle_direction.getY();
                    circle_point.z = angle_direction.getZ();
                    circle_line.points.push_back(circle_point);
                }
                circle_viz.markers.push_back(circle_line);
                circle_line.points.clear();
            }
        }
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
        if (!node_handle_.getParam("circle_mode", circle_mode))
            ROS_WARN_STREAM("Parameter circle_mode not set. Using default setting: " << circle_mode);
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
        // waypoint_dist = 3.5;
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
        path_frame_id = path_list.paths[0].header.frame_id;
        // check if the paths exist or have changed
        if (path_list.paths.size() == 0)
        {
            ROS_WARN_STREAM("Path candidates not received!");
            path_receive = false;
            return;
        }
        else
        {
            if (pre_belief_path.size() == 0 && circle_mode == "path")
            {
                ROS_INFO_STREAM("Path belief space initializing...");
                pre_belief_path.resize(path_list.paths.size(), static_cast<float>(1.0/path_list.paths.size()));
                // pre_belief_path = belief_path;
            }
            belief_updateNode::generateGoal(path_list);
            belief_updateNode::linkCircle2Path(path_list);
            path_receive = true;
            // belief_updateNode::PublishResults();
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

    void belief_updateNode::linkCircle2Path(voronoi_msgs_and_types::PathList path_info)
    {
        if (circle_mode == "direction")
        {
            std::vector<tf2::Vector3> path_directions;
            tf2::Vector3 temp_vector;
            for(int i = 0; i < path_info.paths.size(); i++)
            {
                for(int j = 0; j < path_info.paths[i].poses.size(); j++)
                {
                    if (j > 2 || j == path_info.paths[i].poses.size() - 1)
                    {
                        temp_vector.setX(path_info.paths[i].poses[j].pose.position.x - path_info.paths[i].poses[0].pose.position.x);
                        temp_vector.setY(path_info.paths[i].poses[j].pose.position.y - path_info.paths[i].poses[0].pose.position.y);
                        temp_vector.setZ(0);
                        path_directions.push_back(temp_vector);
                        break;
                    }
                }
            }
            // distribute the probabilities of circle sectors to new paths based on the angle difference
            waypoint_belief.distribution.clear();
            waypoint_belief.distribution.resize(path_info.paths.size(), 0.0);
            std::vector<float> angle_diff;
            float angle_sum = 0;
            for (int i = 0; i < belief_circle.size(); i++)
            {
                for (int j = 0; j < path_info.paths.size(); j++)
                {
                    float angle_vec_diff = M_PI - circle_global_direction[i].angle(path_directions[j]);
                    angle_sum += angle_vec_diff;
                    angle_diff.push_back(angle_vec_diff);
                }
                for (int j = 0; j < path_info.paths.size(); j++)
                {
                    waypoint_belief.distribution[j] += belief_circle[i] * angle_diff[j] / angle_sum;
                }
                angle_diff.clear();
                angle_sum = 0;
            }
            path_directions.clear();
        }
        else if (circle_mode == "path")
        {
            // distribute the probabilities of circle sectors to new paths based on the angle difference
            waypoint_belief.distribution.clear();
            waypoint_belief.distribution.resize(path_info.paths.size(), 0.0);
            std::vector<float> angle_diff;
            float angle_sum = 0;
            for (int i = 0; i < pre_belief_path.size(); i++)
            {
                // std::cout << "Before pre path " << pre_belief_path[i] << std::endl;
                for (int j = 0; j < path_info.paths.size(); j++)
                {
                    float angle_vec_diff = M_PI - pre_goal_local_direction[i].angle(goal_local_direction[j]);
                    angle_sum += angle_vec_diff;
                    angle_diff.push_back(angle_vec_diff);
                }
                for (int j = 0; j < path_info.paths.size(); j++)
                {
                    // std::cout << "Before new path " << waypoint_belief.distribution[j] << std::endl;
                    waypoint_belief.distribution[j] += pre_belief_path[i] * angle_diff[j] / angle_sum;
                    // std::cout << "After new path " << waypoint_belief.distribution[j] << std::endl;
                }
                angle_diff.clear();
                angle_sum = 0;
            }
            // std::cout << "before clear" << std::endl;
            // for (int i = 0; i < belief_path.size(); i++)
            // {
            //     std::cout << "Belief " << i << ", " << belief_path[i] << std::endl;
            // }
            // pre_belief_path.clear();
            // pre_belief_path = waypoint_belief.distribution;
            // pre_goal_local_direction.clear();
            // pre_goal_local_direction = goal_local_direction;
            // belief_path.clear();
            // belief_path = waypoint_belief.distribution;
            // std::cout << "after assingment" << std::endl;
            // for (int i = 0; i < pre_belief_path.size(); i++)
            // {
            //     std::cout << "Path " << path_info.paths[i].header.seq << ", " << pre_belief_path[i] << std::endl;
            // }
        }
    }

    void belief_updateNode::generateGoal(voronoi_msgs_and_types::PathList path_info)
    {
        waypoint_belief.waypoints.poses.clear();
        waypoint_belief.waypoints.poses.resize(path_info.paths.size());
        waypoint_belief.waypoints.header.frame_id = base_frame_id;
        // transform from map to base_link
        tf_buffer.clear();
        geometry_msgs::TransformStamped map2localTransform;
        try
        {
            map2localTransform = tf_buffer.lookupTransform(base_frame_id, path_frame_id, ros::Time(0), ros::Duration(tf_buffer_timeout));
        }
        catch (tf2::TransformException &Exception) 
        {
            ROS_ERROR_STREAM(Exception.what());
        }

        float acmlt_dist = 0;
        geometry_msgs::Pose generated_goal;
        
        tf2::Vector3 goal_direction;
        tf2::Quaternion goal_quat;
        tf2Scalar yaw;
        goal_local_direction.clear();
        goal_local_direction.resize(path_info.paths.size());
        bool direction_receive = false;
        bool waypoint_receive = false;
        for (int i = 0; i < path_info.paths.size(); i++)
        {
            for (int j = 0; j < path_info.paths[i].poses.size(); j++)
            {
                // std::cout << "path " << i << "distance " << acmlt_dist << std::endl;
                tf2::doTransform<geometry_msgs::PoseStamped>(path_info.paths[i].poses[j], path_info.paths[i].poses[j], map2localTransform);
                if (j > 2 || j == path_info.paths[i].poses.size() - 1)
                {
                    goal_local_direction[i] = tf2::Vector3(path_info.paths[i].poses[j].pose.position.x, path_info.paths[i].poses[j].pose.position.y, 0);
                    direction_receive = true;
                }
                
                if (j == 0)
                {
                    acmlt_dist += belief_updateNode::calDistance(0, 0, path_info.paths[i].poses[0].pose.position.x, path_info.paths[i].poses[0].pose.position.y);
                }
                else
                {
                    if (j == path_info.paths[i].poses.size() - 1)
                    {
                        generated_goal = path_info.paths[i].poses[j].pose;
                        goal_direction = tf2::Vector3(path_info.paths[i].poses[j].pose.position.x - path_info.paths[i].poses[j - 1].pose.position.x,
                            path_info.paths[i].poses[j].pose.position.y - path_info.paths[i].poses[j - 1].pose.position.y, 0);
                        waypoint_receive = true;
                    }
                    else
                    {
                        if (!waypoint_receive)
                        {
                            acmlt_dist += belief_updateNode::calDistance(path_info.paths[i].poses[j - 1].pose.position.x, path_info.paths[i].poses[j - 1].pose.position.y,
                            path_info.paths[i].poses[j].pose.position.x, path_info.paths[i].poses[j].pose.position.y);
                            if (acmlt_dist >= waypoint_dist)
                            {
                                generated_goal = path_info.paths[i].poses[j].pose;
                                goal_direction = tf2::Vector3(path_info.paths[i].poses[j].pose.position.x - path_info.paths[i].poses[j - 1].pose.position.x,
                                    path_info.paths[i].poses[j].pose.position.y - path_info.paths[i].poses[j - 1].pose.position.y, 0);
                                waypoint_receive = true;
                            }
                        }
                    }
                }

                if (direction_receive && waypoint_receive)
                {
                    break;
                }
            }
            yaw = goal_direction.angle(tf2::Vector3(1, 0, 0));
            if (goal_direction.getY() < 0)
            {
                yaw = -yaw;
            }
            goal_quat.setRPY(0, 0, yaw);
            tf2::convert(goal_quat, generated_goal.orientation);
            waypoint_belief.waypoints.poses[i] = generated_goal;
            acmlt_dist = 0;
            direction_receive = false;
            waypoint_receive = false;
            // std::cout << "Goal " << i << ",x " << generated_goal.position.x << ",y " << generated_goal.position.y << ",z " << generated_goal.position.z << std::endl;
        }
        goal_list.poses.clear();
        goal_list = waypoint_belief.waypoints;
        if (pre_goal_local_direction.size() == 0)
        {
            ROS_INFO_STREAM("Path local directions initializing...");
            pre_goal_local_direction = goal_local_direction;
        }
        // std::cout << "belief size " << belief_path.size() << std::endl;
        // std::cout << "goal size " << goal_list.poses.size() << std::endl;
        // for (int i = 0; i < goal_list.poses.size(); i++)
        // {
        //     std::cout << "Goal " << i << ",x " << goal_list.poses[i].position.x << ",y " << goal_list.poses[i].position.y 
        //         << ",dist " << belief_updateNode::calDistance(0, 0, goal_list.poses[i].position.x, goal_list.poses[i].position.y) 
        //         << ",ID " << path_info.paths[i].header.seq << std::endl;
        // }
        // for (int i = 0; i < goal_local_direction.size(); i++)
        // {
        //     std::cout << "Path " << path_info.paths[i].header.seq << ",x " << goal_local_direction[i].getX() << ",y " << goal_local_direction[i].getY() << std::endl;
        // }
    }

    void belief_updateNode::PublishResults()
    {
        if (path_receive)
        {
            goal_publisher_.publish(waypoint_belief);
        }
        else
        {
            goal_publisher_.publish(path_belief_update::WaypointDistribution());
        }
        if (circle_mode == "direction")
            viz_publisher_.publish(circle_viz);
        // for (int i = 0; i < waypoint_belief.distribution.size(); i ++)
        // {
        //     std::cout << "Path " << i << ": " << waypoint_belief.distribution[i] << ", ID: " << path_list.paths[i].header.seq << std::endl;
        // }
        input_receive = false;
        path_receive = false;
        odom_receive = false;
        // waypoint_belief.distribution.clear();
        // waypoint_belief.waypoints.poses.clear();
    }

    float belief_updateNode::calRotationValue(float x_input, float y_input, tf2::Vector3 sector_direction)
    {
        tf2::Vector3 agent_heading(1, 0, 0);
        tf2::Vector3 point2goal(x_input, y_input, 0);
        float V_current = agent_heading.angle(sector_direction);
        float V_goal;
        if (x_input == 0 && y_input == 0)
        {
            V_goal = M_PI;
        }
        else
        {
            V_goal = sector_direction.angle(point2goal);
        }
        float C_action = belief_updateNode::calRotationActionCost(V_current);
        // ROS_INFO_STREAM("x_input " << x_input << ", y_input " << y_input);
        // ROS_INFO_STREAM("V_current " << V_current << ", C_action " << C_action << ", V_goal " << V_goal);
        return discount_factor * powf(V_goal/M_PI, 1) * M_PI + C_action + V_current;
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
            V_next = M_PI;
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
        float angle_threshold_local = angle_threshold * M_PI / 180;
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
            if (circle_mode == "direction")
            {
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
                tf2::Quaternion rotation_quat;
                tf2::Vector3 sector_direction;
                tf2::convert(map2localTransform.transform.rotation, rotation_quat);
                tf2::Matrix3x3 map2localMatrix(rotation_quat);
                float Q_rotation[belief_circle.size()], Pi_rotation[belief_circle.size()];
                std::vector<float> softmax_goal;
                float sum_rotation = 0;
                for (int i = 0; i < belief_circle.size(); i++)
                {
                    sector_direction = map2localMatrix * circle_global_direction[i];
                    // ROS_INFO_STREAM("Goal " << i + 1);
                    Q_rotation[i] = belief_updateNode::calRotationValue(x_cmd, y_cmd, sector_direction);
                    // ROS_INFO_STREAM("Goal " << i + 1 << ", V value " << V_g[i] << ", Q value " << Q_gu[i] << ", V+Q " << V_g[i] + Q_gu[i]);
                    // sum_rotation += Q_rotation[i];
                    // std::cout << "Goal " << i + 1 << ", Q Value " << Q_rotation[i] << std::endl;
                    Pi_rotation[i] = exp(- Q_rotation[i]);
                    // ROS_INFO_STREAM("G " << i + 1 << ", C(g) " << Q_rotation[i] << ", Pi " << Pi_rotation[i]);
                    sum_rotation += Pi_rotation[i] * belief_circle[i];
                    softmax_goal.push_back(Pi_rotation[i] * belief_circle[i]);
                }
                // std::transform(belief_circle.begin(), belief_circle.end(), std::bind(std::multiplies<float>(), std::placeholders::_1, 1 / sum_belief_circle));
                for (int i = 0; i < belief_circle.size(); i++)
                {
                    // Pi_rotation[i] = exp(- Q_rotation[i]);
                    // Pi_rotation[i] = powf((1 - Q_rotation[i] / sum_rotation), rate_factor);
                    softmax_goal[i] = softmax_goal[i] / sum_rotation;
                }
                // for (int i = 0; i < belief_circle.size(); i ++)
                // {
                //     // ROS_INFO_STREAM("Color Red " << i << ": " << circle_viz.markers[i].color.r);
                //     ROS_INFO_STREAM("Belief " << i << ": " << softmax_goal[i]);
                // }
                // softmax_goal = belief_updateNode::normalize(softmax_goal);
                // belief_circle = belief_updateNode::softMax(softmax_goal);
                belief_circle = belief_updateNode::clipBelief(softmax_goal);
                tf2Scalar yaw, pitch, roll;
                map2localMatrix.getRPY(roll, pitch, yaw);
                // ROS_INFO_STREAM("Yaw " << yaw * 180 / M_PI);
                int index_shift = round(yaw * 18 / M_PI);
                int n = 0;
                for (int i = 0; i < belief_circle.size(); i++)
                {
                    n = i + index_shift;
                    if (i + index_shift < 0)
                        n += belief_circle.size();
                    else if (i + index_shift >= belief_circle.size())
                        n -= belief_circle.size();
                    circle_viz.markers[n].color.g = 1.0 - powf(belief_circle[i], 0.2);
                }
            }
            else if (circle_mode == "path")
            {
                belief_path.clear();
                belief_path.resize(goal_list.poses.size(), 0.0);
                std::vector<float> angle_diff;
                float angle_sum = 0;
                for (int i = 0; i < pre_belief_path.size(); i++)
                {
                    // std::cout << "Before pre path " << pre_belief_path[i] << std::endl;
                    for (int j = 0; j < goal_local_direction.size(); j++)
                    {
                        float angle_vec_diff = M_PI - pre_goal_local_direction[i].angle(goal_local_direction[i]);
                        angle_sum += angle_vec_diff;
                        angle_diff.push_back(angle_vec_diff);
                    }
                    for (int j = 0; j < goal_local_direction.size(); j++)
                    {
                        belief_path[j] += pre_belief_path[i] * angle_diff[j] / angle_sum;
                    }
                    angle_diff.clear();
                    angle_sum = 0;
                }
                pre_belief_path.clear();
                pre_belief_path = belief_path;
                float Q_rotation[goal_list.poses.size()], Pi_rotation[goal_list.poses.size()];
                std::vector<float> softmax_goal;
                float sum_rotation = 0;
                for (int i = 0; i < goal_list.poses.size(); i++)
                {
                    Q_rotation[i] = belief_updateNode::calRotationValue(x_cmd, y_cmd, goal_list.poses[i]);
                    Pi_rotation[i] = exp(- Q_rotation[i]);
                    sum_rotation += Pi_rotation[i] * pre_belief_path[i];
                    softmax_goal.push_back(Pi_rotation[i] * pre_belief_path[i]);
                }
                for (int i = 0; i < goal_list.poses.size(); i++)
                {
                    softmax_goal[i] = softmax_goal[i] / sum_rotation;
                }
                pre_belief_path = belief_updateNode::clipBelief(softmax_goal);
                pre_goal_local_direction.clear();
                pre_goal_local_direction = goal_local_direction;
                // transfer the probability distribution back to the circle
                // belief_circle.clear();
                // belief_circle.resize(36, 0.0);
                // std::vector<float> angle_diff;
                // float angle_sum = 0;
                // for (int i = 0; i < belief_path.size(); i++)
                // {
                //     // std::cout << "Goal " << i << " " << belief_path[i] << ",x " << goal_list.poses[i].position.x << ",y " << goal_list.poses[i].position.y << std::endl;
                //     // std::cout << "Vector " << i << ",x " << goal_local_direction[i].getX() << ",y " << goal_local_direction[i].getY() << std::endl;
                //     // tf2::Vector3 goal_direction(goal_list.poses[i].position.x, goal_list.poses[i].position.y, 0);
                //     for (int j = 0; j < belief_circle.size(); j++)
                //     {
                //         sector_direction = map2localMatrix * circle_global_direction[j];
                //         // std::cout << "Angle " << angle_circle[j] * 180 / M_PI << ": " << sector_direction.getX() << " " << sector_direction.getY() << std::endl;
                //         float angle_vec_diff = M_PI - goal_local_direction[i].angle(sector_direction);
                //         angle_sum += angle_vec_diff;
                //         angle_diff.push_back(angle_vec_diff);
                //     }
                //     for (int j = 0; j < belief_circle.size(); j++)
                //     {
                //         belief_circle[j] += belief_path[i] * angle_diff[j] / angle_sum;
                //         // std::cout << "Angle " << angle_circle[j] * 180 / M_PI << ": " << belief_circle[j] << std::endl;
                //     }
                //     angle_diff.clear();
                //     angle_sum = 0;
                // }
                // belief_circle = belief_updateNode::clipBelief(belief_circle);
            }
            // std::vector<float>::iterator max_iterator = std::max_element(belief_circle.begin(), belief_circle.end());
            // int max_index = max_iterator - belief_circle.begin();
            // std::cout << "Angle " << angle_circle[max_index] * 180 / M_PI << ": " << belief_circle[max_index] << std::endl;
            // float test = 0;
            // for (int i = 0; i < belief_circle.size(); i ++)
            // {
            //     // ROS_INFO_STREAM("Color Red " << i << ": " << circle_viz.markers[i].color.r);
            //     test += belief_circle[i];
            //     std::cout << "Angle " << angle_circle[i] * 180 / M_PI << ": " << belief_circle[i] << ",cumulation " << test << std::endl;
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
