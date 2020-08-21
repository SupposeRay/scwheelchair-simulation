#include "shared_dwa/shared_DWA_node.h"
// ROS
#include <ros/ros.h>
// tf
#include <tf/transform_datatypes.h>
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
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace shared_DWA
{
    // Constructor
    shared_DWANode::shared_DWANode(ros::NodeHandle &node_handle)
    :   node_handle_(node_handle)
    {
        if (!readParameters())
        {
            ROS_ERROR("Could not load parameters.");
            ros::requestShutdown();
        }

        // Subscribers & Publishers
        scan_subscriber_ = node_handle_.subscribe("/scan", 1, &shared_DWANode::scanCallback, this);
        odom_subscriber_ = node_handle_.subscribe("/encoders/odom", 1, &shared_DWANode::odomCallback, this);
        cmd_subscriber_ = node_handle_.subscribe("/user/cmd_vel", 1, &shared_DWANode::cmdCallback, this);
        vel_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/shared_dwa/cmd_vel", 1);
        // path_publisher_ = node_handle_.advertise<nav_msgs::Path>("/shared_dwa/path", 1);
        cddt_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/path", 1);
        line_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/final_path", 1);
        ucmd_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization/user_path", 1);
        timer_ = node_handle_.createTimer(ros::Duration(publish_interval), &shared_DWANode::timerCallback, this);

        w_max = w_max_robot;
        // initialize the final published twist
        dwa_twist.linear.x = 0;
        dwa_twist.linear.y = 0;
        dwa_twist.linear.z = 0;
        dwa_twist.angular.x = 0;
        dwa_twist.angular.y = 0;
        dwa_twist.angular.z = 0;

        // calculate the total sample time
        // sample_time = std::max(- v_max_robot / v_dclrt, - w_max_robot / w_dclrt);

        // number of samples
        sample_number = floor((sample_time / sample_interval));
        // visualization markers
        candidate_samples.header.frame_id = base_frame_id;
        candidate_samples.ns = "visualization";
        candidate_samples.action = visualization_msgs::Marker::ADD;
        candidate_samples.pose.orientation.w = 1.0;
        candidate_samples.id = 0;
        candidate_samples.type = visualization_msgs::Marker::SPHERE_LIST;
        // POINTS markers use x and y scale for width/height respectively
        candidate_samples.scale.x = 0.05;
        candidate_samples.scale.y = 0.05;
        candidate_samples.scale.z = 0.05;
        candidate_samples.color.g = 1.0;
        candidate_samples.color.a = 0.7;

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
    }
    // Destructor
    shared_DWANode::~shared_DWANode()
    {}
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
        if (!node_handle_.getParam("v_dclrt", v_dclrt))
            ROS_WARN_STREAM("Parameter v_dclrt not set. Using default setting: " << v_dclrt);
        if (!node_handle_.getParam("w_acclrt", w_acclrt))
            ROS_WARN_STREAM("Parameter w_acclrt not set. Using default setting: " << w_acclrt);
        if (!node_handle_.getParam("w_dclrt", w_dclrt))
            ROS_WARN_STREAM("Parameter w_dclrt not set. Using default setting: " << w_dclrt);
        if (!node_handle_.getParam("v_max_robot", v_max_robot))
            ROS_WARN_STREAM("Parameter v_max_robot not set. Using default setting: " << v_max_robot);
        if (!node_handle_.getParam("v_min_robot", v_min_robot))
            ROS_WARN_STREAM("Parameter v_min_robot not set. Using default setting: " << v_min_robot);
        if (!node_handle_.getParam("w_max_robot", w_max_robot))
            ROS_WARN_STREAM("Parameter w_max_robot not set. Using default setting: " << w_max_robot);
        if (!node_handle_.getParam("footprint_mode", footprint_mode))
            ROS_WARN_STREAM("Parameter footprint_mode not set. Using default setting: " << footprint_mode);
        if (!node_handle_.getParam("fixed_frame_id", fixed_frame_id))
            ROS_WARN_STREAM("Parameter fixed_frame_id not set. Using default setting: " << fixed_frame_id);
        if (!node_handle_.getParam("base_frame_id", base_frame_id))
            ROS_WARN_STREAM("Parameter base_frame_id not set. Using default setting: " << base_frame_id);
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
                for (int i = 0; i < temp_list.size(); i = i + 2)
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
            temp_list.clear();
            temp_list.shrink_to_fit();
        }
        else
            return false;
        ROS_INFO_STREAM("Complete loading parameters.");
        return true;
    }    
    void shared_DWANode::scanCallback(const sensor_msgs::LaserScan &msg_scan)
    {
        lidar_scan = msg_scan;
        scan_receive = true;
        // ROS_INFO_STREAM("Scan received.");
    }
    void shared_DWANode::odomCallback(const nav_msgs::Odometry &msg_odom)
    {
        v_agent = msg_odom.twist.twist.linear.x;
        w_agent = msg_odom.twist.twist.angular.z;
        // v_agent = 0.6;
        // w_agent = 0;
        v_agent = round(v_agent * 100) / 100;
        w_agent = round(w_agent * 100) / 100;
        odom_receive = true;
        // ROS_INFO_STREAM("Odom received.");
    }
    void shared_DWANode::cmdCallback(const geometry_msgs::Twist &msg_cmd)
    {
        v_cmd = msg_cmd.linear.x;
        w_cmd = msg_cmd.angular.z;
        t_now = ros::Time::now();
        candidate_samples.header.stamp = t_now;
        cmd_receive = true;
        // ROS_INFO_STREAM("Command received.");
    }
    void shared_DWANode::timerCallback(const ros::TimerEvent&)
    {
        // ROS_INFO_STREAM("Timer received.");
        // ROS_INFO_STREAM("cmd " << cmd_receive << ", odom " << odom_receive << ", scan " << scan_receive);
        if (cmd_receive && odom_receive && scan_receive)
        {
            if ((v_cmd == 0 && w_cmd == 0) || (v_cmd * v_agent < 0))
            {
                // ROS_INFO_STREAM("v_agent " << v_agent << ", v_cmd " << v_cmd);
                // ROS_INFO_STREAM("Triggered.");
                dwa_twist.linear.x = 0;
                dwa_twist.angular.z = 0;                
            }
            else
            {
                v_max = std::min(fabs(v_cmd), fabs(v_max_robot));
                v_min = - std::min(fabs(v_cmd), fabs(v_min_robot));
                // w_max = std::min(fabs(w_cmd), fabs(w_max_robot));
                shared_DWANode::dynamicWindow();
                shared_DWANode::selectVelocity();
            }
            publishResults();
        }       
    }
    void shared_DWANode::publishResults()
    {
        vel_publisher_.publish(dwa_twist);
        cddt_publisher_.publish(candidate_samples);
        line_publisher_.publish(final_line);
        ucmd_publisher_.publish(user_line);
        candidate_samples.points.clear();
        final_line.points.clear();
        user_line.points.clear();
        dynamic_window.release();
        cost_window.release();
        // ROS_DEBUG_STREAM("Twist published!");
        ROS_DEBUG_STREAM("Linear " << dwa_twist.linear.x);
        ROS_DEBUG_STREAM("Angular " << dwa_twist.angular.z);
        // ROS_DEBUG_STREAM("v_cmd " << v_cmd);
        // ROS_DEBUG_STREAM("w_cmd " << w_cmd);
    }
    void shared_DWANode::dynamicWindow()
    {
        v_agent = std::max(std::min(v_max_robot, v_agent), v_min_robot);
        w_agent = std::max(std::min(w_max_robot, w_agent), -w_max_robot);
        // ROS_DEBUG_STREAM("v_agent " << v_agent << ", w_agent " << w_agent);
        float v_agent_abs = fabs(v_agent);
        // compute the boudaries of dynamic window
        float v_upper, v_lower, w_left, w_right;
        if (v_agent <= 0 && v_cmd < 0)
        {
            agent_forward = false;
        }
        else
        {
            agent_forward = true;
        }
        float v_bound = agent_forward ? v_max : fabs(v_min);
        v_upper = std::min(v_agent_abs + v_acclrt * sample_interval, v_bound);
        v_lower = std::min(std::max(v_agent_abs + v_dclrt * sample_interval, static_cast<float>(0)), v_upper);
        w_left = std::min(w_agent + w_acclrt * sample_interval, w_max);
        w_right = std::max(w_agent - w_acclrt * sample_interval, -w_max);
        // discretize the dynamic window w.r.t. v_sample and w_sample
        float row_increment = fabs(v_upper - v_lower) / static_cast<float>(v_sample);
        float col_increment = fabs(w_right - w_left) / static_cast<float>(w_sample);
        // initialize the dynamic window and cost window
        // dynamic window channel 1 is linear velocity
        // dynamic window channel 2 is angular velocity
        // cost window is the cost of all samples
        dynamic_window.create(v_sample + 1, w_sample + 1, CV_32FC2);
        cost_window.create(v_sample + 1, w_sample + 1, CV_32F);
        // heading_window.create(v_sample + 1, w_sample + 1, CV_32F);
        // clearance_window.create(v_sample + 1, w_sample + 1, CV_32F);
        // velocity_window.create(v_sample + 1, w_sample + 1, CV_32F);
        float heading_ = 0, velocity_ = 0, clearance_ = 0;
        // ROS_DEBUG_STREAM(v_upper << " " << v_lower << " " << w_left << " " << w_right);
        // ROS_DEBUG_STREAM("row_increment " << row_increment << ", col_increment " << col_increment);
        
        for (int i = 0; i < v_sample + 1; i++)
        {
            for (int j = 0; j < w_sample + 1; j++)
            {
                // ROS_DEBUG_STREAM("i " << i << ", j " << j);
                v_dw = v_upper - i * row_increment;
                w_dw = w_left - j * col_increment;
                // ROS_DEBUG_STREAM("v_dw " << v_dw << ", w_dw " << w_dw);
                v_dw = round(v_dw * 1000) / 1000;
                w_dw = round(w_dw * 1000) / 1000;
                dynamic_window.at<cv::Vec2f>(i, j)[0] = v_dw;
                dynamic_window.at<cv::Vec2f>(i, j)[1] = w_dw;
                // ROS_DEBUG_STREAM("v_dw " << v_dw << ", w_dw " << w_dw);
                clearance_ = shared_DWANode::calDist2Collision(v_dw, w_dw, sample_interval, sample_time);
                // ROS_INFO_STREAM("v_cmd " <<  v_cmd << ", w_cmd " << w_cmd);
                heading_ = shared_DWANode::calHeading(v_dw, w_dw, v_cmd, w_cmd);
                // ROS_DEBUG_STREAM("heading_ " << heading_);
                if (fabs(v_cmd) == 0)
                    velocity_ = 0;
                else
                    velocity_ = fabs(v_dw - v_cmd) / v_cmd;
                // ROS_DEBUG_STREAM("velocity_ " << velocity_);
                cost_window.at<float>(i, j) = 1 - clearance_ + clearance_ * (weight_h * heading_ + weight_v * velocity_);
                // heading_window.at<float>(i, j) = heading_;
                // velocity_window.at<float>(i, j) = velocity_;
                // clearance_window.at<float>(i, j) = clearance_;
                // cost_window.at<float>(i, j) = heading_;
                // ROS_DEBUG_STREAM("clearance_ " << clearance_);
            }
        }
        // ROS_DEBUG_STREAM("Window generated.");
        window_generate = true;
    }

    float shared_DWANode::calDist2Collision(float v_dw, float w_dw, float sample_interval, float sample_time)
    {
        // ROS_DEBUG_STREAM("v_dw " << v_dw << ", w_dw " << w_dw);
        if (!agent_forward)
        {
            v_dw *= -1;
        }
        // ROS_INFO_STREAM("agent_forward " << agent_forward);
        geometry_msgs::Point point_candidate;
        float x = 0, y = 0, theta = 0;
        float linear_dist = 0, angular_dist = 0, v_collision = 0, w_collision = 0;
        for (int i = 0; i < sample_number; i++)
        {
            // ROS_DEBUG_STREAM("i " << i);
            x += v_dw * cos(theta) * sample_interval;
            y += v_dw * sin(theta) * sample_interval;
            theta += w_dw * sample_interval;
            // ROS_DEBUG_STREAM("x " << x << ", y " << y << ", theta " << theta);
            point_candidate.x = x;
            point_candidate.y = y;
            point_candidate.z = 0;
            if (shared_DWANode::checkCollision(x, y, theta, footprint_mode))
            {
                linear_dist = (i + 1) * sample_interval * v_dw;
                angular_dist = (i + 1) * sample_interval * w_dw;
                v_collision = sqrt(2 * fabs(v_dclrt) * fabs(linear_dist));
                w_collision = sqrt(2 * fabs(w_dclrt) * fabs(angular_dist));
                // ROS_INFO_STREAM("v_dw " << v_dw << " v_collision " << v_collision << " w_dw " << w_dw << " w_collision " << w_collision);
                if (fabs(v_dw) >= v_collision || (w_dw != 0 && fabs(w_dw) >= w_collision))
                {
                    return 0;
                }
                else
                {
                    // dist2collision = std::min((v_collision - v_dw) / v_collision, (w_collision - w_dw) / w_collision);
                    return static_cast<float>(i + 1) / static_cast<float>(sample_number);
                    // ROS_INFO_STREAM("i + 1 " << i + 1 << " sample_number " << sample_number << " dist2collision " << dist2collision);
                }
                // ROS_DEBUG_STREAM("dist2collision " << dist2collision);
                break;
            }
            candidate_samples.points.push_back(point_candidate);
            if (i + 1 == sample_number)
            {
                return 1;
                // ROS_DEBUG_STREAM("dist2collision " << dist2collision);
            }
        }
        // ROS_DEBUG_STREAM("Trajectory Generated");
    }

    bool shared_DWANode::checkCollision(float x_check, float y_check, float theta_check, std::string footprint_mode)
    {
        if (footprint_mode == "radius")
        {
            float x_scan = 0, y_scan = 0;
            for (int k = 0; k < lidar_scan.ranges.size(); k++)
            {
                // ROS_DEBUG_STREAM("x_scan " << x_scan << ", y_scan " << y_scan);
                x_scan = lidar_scan.ranges[k] * cos(lidar_scan.angle_min + k * lidar_scan.angle_increment);
                y_scan = lidar_scan.ranges[k] * sin(lidar_scan.angle_min + k * lidar_scan.angle_increment);
                float dist_r = calDistance(x_check, y_check, x_scan, y_scan);
                if (dist_r <= r_collision)
                {
                    // ROS_DEBUG_STREAM("Collision!");
                    return true;
                    break;
                }
            }
            return false;
        }
        else if (footprint_mode == "rectangle")
        {
            cv::Point2f A(0, 0), B(0, 0), C(0, 0), D(0, 0), M(0, 0);
            cv::Vec2f AB, BC, AM, BM;
            float dotABAB = 0, dotBCBC = 0, dotABAM = 0, dotBCBM = 0;
            A.x = x_check + rectangle_point[0].x * cos(theta_check) - rectangle_point[0].y * sin(theta_check);
            A.y = y_check + rectangle_point[0].y * cos(theta_check) + rectangle_point[0].x * sin(theta_check);
            B.x = x_check + rectangle_point[1].x * cos(theta_check) - rectangle_point[1].y * sin(theta_check);
            B.y = y_check + rectangle_point[1].y * cos(theta_check) + rectangle_point[1].x * sin(theta_check);
            C.x = x_check + rectangle_point[2].x * cos(theta_check) - rectangle_point[2].y * sin(theta_check);
            C.y = y_check + rectangle_point[2].y * cos(theta_check) + rectangle_point[2].x * sin(theta_check);
            D.x = x_check + rectangle_point[3].x * cos(theta_check) - rectangle_point[3].y * sin(theta_check);
            D.y = y_check + rectangle_point[3].y * cos(theta_check) + rectangle_point[3].x * sin(theta_check);

            AB = cv::Vec2f(B.x - A.x, B.y - A.y);
            BC = cv::Vec2f(C.x - B.x, C.y - B.y);

            dotABAB = calDotproduct(AB, AB);
            dotBCBC = calDotproduct(BC, BC);

            for (int k = 0; k < lidar_scan.ranges.size(); k++)
            {
                M.x = lidar_scan.ranges[k] * cos(lidar_scan.angle_min + k * lidar_scan.angle_increment);
                M.y = lidar_scan.ranges[k] * sin(lidar_scan.angle_min + k * lidar_scan.angle_increment);

                AM = cv::Vec2f(M.x - A.x, M.y - A.y);
                BM = cv::Vec2f(M.x - B.x, M.y - B.y);

                dotABAM = calDotproduct(AB, AM);
                dotBCBM = calDotproduct(BC, BM);

                if (0 <= dotABAM <= dotABAB && 0 <= dotBCBM <= dotBCBC)
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
        return v1[0] * v2 [0] + v1[1] * v2[1];
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
        v_current = std::max( fabs(v_current / v_max_robot), (float)0.0001);
        w_current = w_current / w_max_robot;
        v_command = fabs(v_command) / v_max_robot;
        w_command = w_command / w_max_robot;
        tf::Vector3 V_current(v_current, w_current, 0);
        tf::Vector3 V_command(v_command, w_command, 0);
        // ROS_INFO_STREAM("V_command.X " <<  V_command.getX() << ", V_command.Y " << V_command.getY());
        // ROS_INFO_STREAM("V_current.X " <<  V_current.getX() << ", V_current.Y " << V_current.getY());
        return V_current.angle(V_command) / PI;
    }
    
    void shared_DWANode::selectVelocity()
    {
        if (window_generate)
        {
            double min_cost = 0;
            int min_idx[2] = {};
            cv::minMaxIdx(cost_window, &min_cost, NULL, min_idx, NULL);
            // ROS_INFO_STREAM("cost_window " << std::endl << cost_window);
            // ROS_INFO_STREAM("heading_window " << std::endl << heading_window);
            // ROS_INFO_STREAM("velocity_window " << std::endl << velocity_window);
            // ROS_INFO_STREAM("clearance_window " << std::endl << clearance_window);
            // ROS_INFO_STREAM("dynamic_window " << std::endl << dynamic_window);
            // get the final v and w after Dynamic Window Approach
            // ROS_INFO_STREAM("row " <<  min_idx[0] << ", col " << min_idx[1]);
            if (cost_window.at<float>(min_idx[0],min_idx[1]) >= 1)
            {
                dwa_twist.linear.x = 0;
                dwa_twist.angular.z = 0;
            }
            else
            {
                dwa_twist.linear.x = (2 * static_cast<float>(agent_forward) - 1) * dynamic_window.at<cv::Vec2f>(min_idx[0],min_idx[1])[0];
                dwa_twist.angular.z = dynamic_window.at<cv::Vec2f>(min_idx[0],min_idx[1])[1];
            }
            geometry_msgs::Point final_line_point, user_line_point;
            final_line_point.x = user_line_point.x = 0;
            final_line_point.y = user_line_point.y = 0;
            final_line_point.z = user_line_point.z = 0;
            float theta_final = 0, theta_user = 0;
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
            }
            // ROS_INFO_STREAM("linear " <<  dwa_twist.linear.x << ", angular " << dwa_twist.angular.z);
        }
        else
            ROS_ERROR_STREAM("Error! Dynamic window is not generated.");
    }
}
