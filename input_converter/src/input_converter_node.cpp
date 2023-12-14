#include "input_converter/input_converter_node.h"

namespace input_converter
{
    // Constructor
    input_converterNode::input_converterNode(ros::NodeHandle &node_handle)
    :   node_handle_(node_handle)
    {
        if (!readParameters())
        {
            ROS_ERROR_STREAM("Could not load parameters.");
            ros::requestShutdown();
        }

        // Subscribers & Publishers
        joystick_subscriber_ = node_handle_.subscribe("/joystick_calib", 1, &input_converterNode::joystickCallback, this);

        vel_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/input_converter/cmd_vel", 1);

        timer_ = node_handle_.createTimer(ros::Duration(publish_interval), &input_converterNode::timerCallback, this);

	    cmd_twist.linear.x = 0;
        cmd_twist.angular.z = 0;
        old_cmd_twist.linear.x = 0;
        old_cmd_twist.angular.z = 0;
    }
    // Destructor
    input_converterNode::~input_converterNode()
    {}
    //Public Member Functions

    //Private Member Functions
    bool input_converterNode::readParameters()
    {
        ROS_INFO_STREAM("Loading parameters.....");
        if (!node_handle_.getParam("publish_interval", publish_interval))
            ROS_WARN_STREAM("Parameter publish_interval not set. Using default setting: " << publish_interval);
        if (!node_handle_.getParam("filter_type", filter_type))
            ROS_WARN_STREAM("Parameter filter_type not set. Using default setting: " << filter_type);
        if (!node_handle_.getParam("buffer_size", buffer_size_))
            ROS_WARN_STREAM("Parameter buffer_size not set. Using default setting: " << buffer_size_);
        if (!node_handle_.getParam("linear_scale", linear_scale))
            ROS_WARN_STREAM("Parameter linear_scale not set. Using default setting: " << linear_scale);
        if (!node_handle_.getParam("angular_scale", angular_scale))
            ROS_WARN_STREAM("Parameter angular_scale not set. Using default setting: " << angular_scale);
	    if (!node_handle_.getParam("linear_acc", linear_acc))
            ROS_WARN_STREAM("Parameter linear_acc not set. Using default setting: " << linear_acc);
        if (!node_handle_.getParam("angular_acc", angular_acc))
            ROS_WARN_STREAM("Parameter angular_acc not set. Using default setting: " << angular_acc);

        if (filter_type == "None")
        {}
        else if (filter_type == "Mean Filter")
        {
            add_filter = true;
        }
        else
            return false;

        ROS_INFO_STREAM("Complete loading parameters.");
        return true;
    }

    void input_converterNode::joystickCallback(const geometry_msgs::Point &msg_joystick)
    {
        input_joystick.x = msg_joystick.x;
        input_joystick.y = msg_joystick.y;
        // input_joystick.x = round(msg_joystick.x * 100) / 100;
        // input_joystick.y = round(msg_joystick.y * 100) / 100;
        
        if (add_filter)
        {
            input_converterNode::meanFilter(input_joystick);
        }

        // input_joystick.x = input_joystick.x > 0 ? input_joystick.x : input_joystick.x / 0.91;
        // input_joystick.y = input_joystick.y > 0 ? input_joystick.y : input_joystick.y / 0.88;

        if (fabs(input_joystick.x) >= 0.05)
        {
            cmd_twist.linear.x = linear_scale * input_joystick.x;
        }
        else
        {
            cmd_twist.linear.x = 0;
        }
        if (fabs(input_joystick.y) >= 0.05)
        {
            cmd_twist.angular.z = angular_scale * input_joystick.y;
        }
        else
        {
            cmd_twist.angular.z = 0;
        }
        // cmd_twist.angular.z = (cmd_twist.linear.x >= - 0.1) ? angular_scale * input_joystick.y : - angular_scale * input_joystick.y;
        joystick_receive = true;
    }

    void input_converterNode::timerCallback(const ros::TimerEvent&)
    {
        if (joystick_receive)
        {
            publishResults();
        }
        else
        {
            ROS_WARN_STREAM("No input source.");
            ros::spinOnce();
        }
    }

    void input_converterNode::publishResults()
    {
	    if (fabs(cmd_twist.linear.x  - old_cmd_twist.linear.x) > publish_interval * linear_acc)
        {
            if (cmd_twist.linear.x  > old_cmd_twist.linear.x)
            {
                cmd_twist.linear.x = old_cmd_twist.linear.x + publish_interval * linear_acc;
            }
            else
            {
                cmd_twist.linear.x = old_cmd_twist.linear.x - publish_interval * linear_acc;
            }
        }
        
        if (fabs(cmd_twist.angular.z  - old_cmd_twist.angular.z) > publish_interval * angular_acc)
        {
            if (cmd_twist.angular.z  > old_cmd_twist.angular.z)
            {
                cmd_twist.angular.z = old_cmd_twist.angular.z + publish_interval * angular_acc;
            }
            else
            {
                cmd_twist.angular.z = old_cmd_twist.angular.z - publish_interval * angular_acc;
            }
        }

        cmd_twist.linear.x = round(cmd_twist.linear.x * 1000) / 1000;
        cmd_twist.angular.z = round(cmd_twist.angular.z * 1000) / 1000;
        
        vel_publisher_.publish(cmd_twist);
	    old_cmd_twist = cmd_twist;
        joystick_receive = false;
    }

    void input_converterNode::meanFilter(geometry_msgs::Point &msg_input)
    {
        buffer_x_.push_back(msg_input.x);
        buffer_y_.push_back(msg_input.y);
        if (buffer_x_.size() > buffer_size_)
        {
            buffer_x_.pop_front();
            buffer_y_.pop_front();
        }
        average_x_ = std::accumulate(buffer_x_.begin(), buffer_x_.end(), 0.0) / buffer_x_.size();
        average_y_ = std::accumulate(buffer_y_.begin(), buffer_y_.end(), 0.0) / buffer_y_.size();

        msg_input.x = average_x_;
        msg_input.y = average_y_;
    }
}
