#include "trajectory_generator/trajectory_generator_node.h"

namespace trajectory_generator
{
    // Constructor
    TrajectoryGeneratorNode::TrajectoryGeneratorNode(ros::NodeHandle &node_handle)
    :   node_handle_(node_handle),
        publish_frequency_(0.1),
        time_interval(0.1),
        path_length(80),
        linear_acclrt(0.1),
        angular_acclrt(0.1)
    {
        // ROS_DEBUG_STREAM("Constructor.");
        /*
        if (!readParameters())
        {
            ROS_ERROR_STREAM("Could not load parameters.");
            ros::requestShutdown();
        }
        */
        // initialize the path length
        agent_trajectory.poses.resize(path_length);
        // Subscribers & Publishers
        cmd_subscriber_ = node_handle_.subscribe("/user/cmd_vel", 1, &TrajectoryGeneratorNode::cmdCallback, this);
        // odom_subscriber_ = node_handle_.subscribe("/odom", 1, &TrajectoryGeneratorNode::odomCallback, this);
        map_subscriber_ = node_handle_.subscribe("/map", 1, &TrajectoryGeneratorNode::mapCallback, this);
        amcl_subscriber_ = node_handle_.subscribe("/amcl_pose", 1, &TrajectoryGeneratorNode::amclCallback, this);
        cost_subscriber_ = node_handle_.subscribe("/move_base/global_costmap/costmap", 1, &TrajectoryGeneratorNode::costCallback, this);
        trajectory_publisher_ = node_handle_.advertise<nav_msgs::Path>("trajectory", 1);
        goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("goal", 1);
        timer_ = node_handle_.createTimer(ros::Duration(publish_frequency_), &TrajectoryGeneratorNode::timerCallback, this);
    }
    // Destructor
    TrajectoryGeneratorNode::~TrajectoryGeneratorNode()
    {}
    //Public Member Functions

    //Private Member Functions
    /*
    bool TrajectoryGeneratorNode::readParameters()
    {
        if(!(
            node_handle_.getParam("publish_frequency", publish_frequency_) &
            node_handle_.getParam("cmd_topic", cmd_topic_)
        ))
            return false;
        ROS_DEBUG_STREAM("Parameters read.");
        return true;
    }
    */

    void TrajectoryGeneratorNode::cmdCallback(const geometry_msgs::Twist::ConstPtr &msg_cmd)
    {
        user_cmd.linear = msg_cmd->linear;
        user_cmd.angular = msg_cmd->angular;
        v_cmd = user_cmd.linear.x;
        w_cmd = user_cmd.angular.z;
        // ROS_INFO_STREAM("v " << v_cmd << ", w " << w_cmd);
        cmd_receive = true;
        TrajectoryGeneratorNode::publishResults();
    }
    void TrajectoryGeneratorNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg_map)
    {

        agent_trajectory.header = msg_map->header;
        agent_goal.header = msg_map->header;
        map_receive = true;
    }
    void TrajectoryGeneratorNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom)
    {
        // ROS_INFO_STREAM("odom_received");
        agent_odom.header = msg_odom->header;
        // agent_trajectory.header = agent_odom.header;
        // agent_goal.header = agent_odom.header;
        agent_odom.pose = msg_odom->pose;
        agent_odom.twist = msg_odom->twist;
        agent_odom.child_frame_id = msg_odom->child_frame_id;
        x_agent = agent_odom.pose.pose.position.x;
        y_agent = agent_odom.pose.pose.position.y;
        z_agent = agent_odom.pose.pose.position.z;
        // v_agent = agent_odom.twist.twist.linear.x;
        // w_agent = agent_odom.twist.twist.angular.z;
        odom_receive = true;
    }
    void TrajectoryGeneratorNode::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg_amcl)
    {
        agent_odom.header = msg_amcl->header;
        // agent_trajectory.header = agent_odom.header;
        // agent_goal.header = agent_odom.header;
        agent_odom.pose = msg_amcl->pose;
        x_agent = agent_odom.pose.pose.position.x;
        y_agent = agent_odom.pose.pose.position.y;
        z_agent = agent_odom.pose.pose.position.z;
        // ROS_INFO_STREAM("x " << x_agent << ", y " << y_agent << ", z " << z_agent);
        // v_agent = agent_odom.twist.twist.linear.x;
        // w_agent = agent_odom.twist.twist.angular.z;
        amcl_receive = true;
    }
    void TrajectoryGeneratorNode::costCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg_cost)
    {
        ROS_INFO_STREAM("map_received");
        local_costmap.info = msg_cost->info;
        local_costmap.data = msg_cost->data;
        local_costmap.header = msg_cost->header;
        map_cols = local_costmap.info.width;
        map_rows = local_costmap.info.height;
        map_resolution = local_costmap.info.resolution;
        origin_x = local_costmap.info.origin.position.x;
        origin_y = local_costmap.info.origin.position.y;
        origin_z = local_costmap.info.origin.position.z;
        cost_receive = true;
        TrajectoryGeneratorNode::cvtMapToGradImg(local_costmap);
    }
    void TrajectoryGeneratorNode::timerCallback(const ros::TimerEvent&)
    {
        TrajectoryGeneratorNode::publishResults();
    }
    void TrajectoryGeneratorNode::cvtMapToGradImg(nav_msgs::OccupancyGrid map_)
    {
        cv::Mat map_img_;
        int Cols = map_.info.width;
        int Rows = map_.info.height;
        map_img_.create(Rows, Cols, CV_8U);
        for (int i = 0; i < Rows; i++)
        {
            for (int j = 0; j < Cols; j++)
            {
                if (round(map_.data[i * Cols + j]) >= 0)
                {
                    map_img_.at<uchar>(i, j) = map_.data[i * Cols + j];
                }
                else
                {
                    map_img_.at<uchar>(i, j) = 255;        
                }
            }
        }
        // cv::Sobel(map_img_, gradimg_x, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
        // cv::Sobel(map_img_, gradimg_y, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7));
        cv::dilate(map_img_, map_img_, element);
        // cv::addWeighted(gradimg_x, 0.5, gradimg_y, 0.5, 0, dstImg);
        // cv::namedWindow("Final Sobel", cv::WINDOW_NORMAL);
        // cv::imshow("Source", map_img_);
        // cv::imshow("Dilate", dltImg);
        // cv::imshow("Sobel", dstImg);
        // cv::imshow("x Sobel", gradimg_x);
        // cv::imshow("y Sobel", gradimg_y);
        // cv::convertScaleAbs(dstImg, dstImg);
        // cv::imshow("Sobel", dstImg);
        cv::Sobel(map_img_, gradimg_x, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
        cv::Sobel(map_img_, gradimg_y, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
        // cv::addWeighted(gradimg_x, 0.5, gradimg_y, 0.5, 0, dstImg);
        // cv::convertScaleAbs(gradimg_y, gradimg_y);
        // cv::imshow("Dilated Sobel", gradimg_y);

        // cv::waitKey(0);      
    }
    void TrajectoryGeneratorNode::generateTrajectory()
    {
        // time cost to accelerate to user command velocities
        // float time_linear = (v_cmd - v_agent) / linear_acclrt;
        // float time_angular = (w_cmd - w_agent) / angular_acclrt;
        x_agent = agent_odom.pose.pose.position.x;
        y_agent = agent_odom.pose.pose.position.y;
        z_agent = agent_odom.pose.pose.position.z;
        // obtain current direction of agent
        tf::Quaternion agent_quat;
        tf::quaternionMsgToTF(agent_odom.pose.pose.orientation, agent_quat);
        // express current direction in matrix form
        tf::Matrix3x3 agent_rotation(agent_quat);
        // express current linear velocity in vector form
        tf::Vector3 agent_velocity(v_cmd, 0, 0);
        agent_velocity = agent_rotation * agent_velocity;
        // declare a cumulative variable to count the total rotation angle
        float rotary_angle = 0;
        // initialize current angular velocity via user command
        float w_current = w_cmd;
        // velocity direction
        tf::Vector3 velocity_direction(1, 0, 0);
        velocity_direction = agent_rotation * velocity_direction;
        // initial direction
        tf::Vector3 default_direction(1, 0, 0);
        // previous velocity
        tf::Vector3 previous_velocity;
        previous_velocity = agent_velocity;
        // determinant of changing direction
        bool change_direction = false;
        // determinant of the status, true is free, false is non-free
        bool current_status = false;
        // start the loop to obtain all the points on the path
        for (int i = 0; i < path_length; i++)
        {
            // ROS_INFO_STREAM("i " << i);
            //     // float dot_product = default_direction.dot(velocity_direction);
            //     // float cross_product = default_direction.cross(velocity_direction).length();
            //     // tf::Matrix3x3 velocity_rotation(dot_product, - cross_product, 0, cross_product, dot_product, 0, 0, 0, 1);
            //     // tf::Quaternion velocity_quat;
            //     // velocity_rotation.getRotation(velocity_quat);
            // // {
            // //     velocity_direction.setValue(1, 0, 0);
            // // }
            // // if the total rotation angle is larger than PI, stops
            if (fabs(rotary_angle) >= 0.95 * PI)
            {
                // velocity_direction = agent_velocity.normalized();
                agent_velocity.setZero();
                w_current = 0;
            }
            
            // find corresponding row and col index of gradient image
            int index_col = ceil((x_agent - origin_x) / map_resolution) - 1;
            int index_row = ceil((y_agent - origin_y) / map_resolution) - 1;
            // deal with the linear acceleration
            // if ((i + 1) * time_interval <= time_linear)
            // {
            //     agent_velocity = agent_velocity * (v_agent + time_interval * linear_acclrt) / v_agent;
            //     v_agent = v_agent + time_interval * linear_acclrt;
            // }
            // else
            // {
            //     agent_velocity = agent_velocity * v_cmd / v_agent;
            // }
            // declare the roation angle
            float Yaw;
            // determine if the current point is near the obstacle

            // situation 1: the point is out of the map boundary
            if (index_row > map_rows - 1 || index_row < 0 || index_col > map_cols - 1 || index_col < 0)
            {
                agent_velocity.setZero();   // set velocity to zero
                Yaw = 0;    // stop rotation
            }
            // situation 2: the point is at free space
            else if (gradimg_x.at<float>(index_row, index_col) == 0 && gradimg_y.at<float>(index_row, index_col) == 0)
            {
                // check if previous step is direction changed step
                if (change_direction)
                {
                    // resume the previous velocity
                    agent_velocity = previous_velocity;
                    velocity_direction = agent_velocity.normalized();
                    Yaw = 0;
                    change_direction = false;
                }
                else
                {
                    Yaw = w_current * time_interval;  // rotation angle increment
                }
                current_status = true;     // current status is free
            }
            // situation 3: the point is near obstacle
            else
            {
                // check if previous step is a free step
                if (current_status)
                {
                    previous_velocity = agent_velocity;     // store the previous velocity
                }
                current_status = false;         // current status is non-free
                change_direction = true;         // triger the determinant
                Yaw = 0;    // stop rotation
                // obtain the gradient direction and compute the vector perpendicular to the gradient
                tf::Vector3 gradient_direction(gradimg_x.at<float>(index_row, index_col), gradimg_y.at<float>(index_row, index_col), 0);

                float velocitygradient_angle = 0;
                velocitygradient_angle = agent_velocity.angle(gradient_direction);

                // // check if the robot's current velocity is approaching or away from the gradient
                // 
                // if (agent_velocity.length() > 0)
                // {
                //     velocitygradient_angle = velocity_direction.angle(gradient_direction);
                // }
                // if (i == 0)
                // {
                //     ROS_INFO_STREAM("velocitygradient_angle " << velocitygradient_angle);
                //     ROS_INFO_STREAM("velocity_direction " << velocity_direction.getX() << " " << velocity_direction.getY() << " " << velocity_direction.getZ());
                // }


                tf::Vector3 perpen_direction1, perpen_direction2;
                if (gradient_direction.getY() != 0)
                {
                    perpen_direction1.setX(1);
                    perpen_direction1.setY(- gradient_direction.getX() / gradient_direction.getY());
                    perpen_direction1.setZ(0);
                    perpen_direction2.setX(-1);
                    perpen_direction2.setY(gradient_direction.getX() / gradient_direction.getY());
                    perpen_direction2.setZ(0);
                }
                else
                {
                    perpen_direction1.setX(0);
                    perpen_direction1.setY(-1);
                    perpen_direction1.setZ(0);
                    perpen_direction2.setX(0);
                    perpen_direction2.setY(1);
                    perpen_direction2.setZ(0);
                }
                perpen_direction1.normalize();
                perpen_direction2.normalize();
                // compute the projection of velocity onto perpendicular vectors
                float Projection1 = agent_velocity.getX()*perpen_direction1.getX() + agent_velocity.getY()*perpen_direction1.getY() + agent_velocity.getZ()*perpen_direction1.getZ();
                float Projection2 = agent_velocity.getX()*perpen_direction2.getX() + agent_velocity.getY()*perpen_direction2.getY() + agent_velocity.getZ()*perpen_direction2.getZ();
                // select the positive value and its corresponding vector as the new direction of velocity
                if (Projection1 > 0)
                {
                    if ((agent_velocity.length() > 0 && velocitygradient_angle <= PI / 2) || agent_velocity.length() == 0)
                    {
                        agent_velocity = Projection1 * perpen_direction1;
                        velocity_direction = agent_velocity.normalized();
                    }
                }
                else if (Projection1 == 0)
                {
                    if ((agent_velocity.length() > 0 && velocitygradient_angle <= PI / 2) || agent_velocity.length() == 0)
                    {
                        agent_velocity.setZero();
                        w_current = 0;
                    }
                }
                else
                {
                    if ((agent_velocity.length() > 0 && velocitygradient_angle <= PI / 2) || agent_velocity.length() == 0)
                    {
                        agent_velocity = Projection2 * perpen_direction2;
                        velocity_direction = agent_velocity.normalized();
                    }
                }
            }
            // deal with the angular acceleration
            // if ((i + 1) * time_interval <= time_angular)
            // {
            //     w_agent = w_agent + time_interval * angular_acclrt;
            // }
            // else
            // {
            //     w_agent = w_cmd;
            // }
            // update x,y,z of agent according to current linear velocity
            x_agent = x_agent + agent_velocity.getX() * time_interval;
            y_agent = y_agent + agent_velocity.getY() * time_interval;
            z_agent = z_agent + agent_velocity.getZ() * time_interval;
            // add current point position to the path
            agent_trajectory.poses[i].header = agent_trajectory.header;
            agent_trajectory.poses[i].pose.position.x = x_agent;
            agent_trajectory.poses[i].pose.position.y = y_agent;
            agent_trajectory.poses[i].pose.position.z = z_agent;
            // update velocity direction after the rotation in this time interval
            tf::Matrix3x3 inter_rotation;
            inter_rotation.setRPY(0, 0, Yaw);
            agent_velocity = inter_rotation * agent_velocity;
            velocity_direction = inter_rotation * velocity_direction;
            // add current point orientation to the path
            float dot_product = default_direction.dot(velocity_direction);
            float velocity_angle = acos(std::max(std::min(dot_product / velocity_direction.length(), 1.0), - 1.0));
            float cross_product = default_direction.getX()*velocity_direction.getY() - velocity_direction.getX()*default_direction.getY();
            if (cross_product < 0)
            {
                velocity_angle = - velocity_angle;
            }
            // ROS_INFO_STREAM("dot_product " << dot_product << ", i " << i);
            // ROS_INFO_STREAM("angle " << velocity_angle * 180 / PI);
            tf::Matrix3x3 velocity_rotation;
            velocity_rotation.setRPY(0, 0, velocity_angle);
            tf::Quaternion velocity_quat;
            velocity_rotation.getRotation(velocity_quat);
            // agent_rotation = inter_rotation * agent_rotation;
            // agent_rotation.getRotation(agent_quat);
            tf::quaternionTFToMsg(velocity_quat, agent_trajectory.poses[i].pose.orientation);
            if (i == path_length - 1)
            {
                agent_goal.pose.position.x = agent_trajectory.poses[i].pose.position.x;
                agent_goal.pose.position.y = agent_trajectory.poses[i].pose.position.y;
                agent_goal.pose.position.z = agent_trajectory.poses[i].pose.position.z;
                agent_goal.pose.orientation = agent_trajectory.poses[i].pose.orientation;
                // ROS_INFO_STREAM("x " << agent_goal.pose.position.x << ", y " << agent_goal.pose.position.y << ", z " << agent_goal.pose.position.z);
            }
            // count total ratation angle
            rotary_angle = rotary_angle + Yaw;
            // ROS_INFO_STREAM(velocity_direction.getZ());
        }
    }

    void TrajectoryGeneratorNode::publishResults()
    {
        // if (cmd_receive && (odom_receive || amcl_receive) && cost_receive)
        if ((odom_receive || amcl_receive) && cost_receive && map_receive)
        {
            TrajectoryGeneratorNode::generateTrajectory();
            trajectory_publisher_.publish(agent_trajectory);
            goal_publisher_.publish(agent_goal);
        }
    }

}