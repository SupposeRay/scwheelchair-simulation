// ROS
#include <ros/ros.h>
// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// Messages
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
// Visualization
// #include <visualization_msgs/Marker.h>
// C++
#include <math.h>
#include <string>
// #include <functional> 
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>

move_base_msgs::MoveBaseActionGoal global_goal;
geometry_msgs::Pose initial_pose, goal_pose, mid_pose;
ros::Publisher goal_publisher;
ros::Subscriber amcl_subscriber;
float publish_interval;
std::string test_world;
std::string end_point;
bool agent_forward;

void initializeGoal()
{
    initial_pose.position.x = 0;
    initial_pose.position.y = 0;
    initial_pose.position.z = 0;
    initial_pose.orientation.x = 0;
    initial_pose.orientation.y = 0;
    initial_pose.orientation.z = 0;
    initial_pose.orientation.w = 1;
    if (test_world == "test")
    {
        initial_pose.position.x = 1.15;
        initial_pose.position.y = 0.44;
        initial_pose.position.z = 0;
        initial_pose.orientation.x = 0;
        initial_pose.orientation.y = 0;
        initial_pose.orientation.z = 0;
        initial_pose.orientation.w = 1;
        goal_pose.position.x = 16.5;
        goal_pose.position.y = 0.3;
        goal_pose.position.z = 0;
        goal_pose.orientation.x = 0;
        goal_pose.orientation.y = 0;
        goal_pose.orientation.z = 0;
        goal_pose.orientation.w = 1;
    }
    else if (test_world == "diamond")
    {
        goal_pose.position.x = -0.4;
        goal_pose.position.y = 10.8;
        goal_pose.position.z = 0;
        goal_pose.orientation.x = 0;
        goal_pose.orientation.y = 0;
        goal_pose.orientation.z = 0;
        goal_pose.orientation.w = 1;
    }
    else if (test_world == "museum")
    {
        goal_pose.position.x = -6.18;
        goal_pose.position.y = 2.31;
        goal_pose.position.z = 0;
        goal_pose.orientation.x = 0;
        goal_pose.orientation.y = 0;
        goal_pose.orientation.z = -0.712;
        goal_pose.orientation.w = 0.702;
    }
    else if (test_world == "hospital")
    {
        mid_pose.position.x = 13.74;
        mid_pose.position.y = 4.07;
        mid_pose.position.z = 0.0;
        mid_pose.orientation.x = 0;
        mid_pose.orientation.y = 0;
        mid_pose.orientation.z = -0.69;
        mid_pose.orientation.w = 0.724;
        goal_pose.position.x = 4.12;
        goal_pose.position.y = 13.45;
        goal_pose.position.z = 0;
        goal_pose.orientation.x = 0;
        goal_pose.orientation.y = 0;
        goal_pose.orientation.z = -1;
        goal_pose.orientation.w = 0.03;
    }
    else if (test_world == "pomdp_test")
    {
        mid_pose.position.x = 13.74;
        mid_pose.position.y = 4.07;
        mid_pose.position.z = 0.0;
        mid_pose.orientation.x = 0;
        mid_pose.orientation.y = 0;
        mid_pose.orientation.z = -0.69;
        mid_pose.orientation.w = 0.724;
        goal_pose.position.x = 4.885;
        goal_pose.position.y = -17.00;
        goal_pose.position.z = 0;
        goal_pose.orientation.x = 0;
        goal_pose.orientation.y = 0;
        goal_pose.orientation.z = 0;
        goal_pose.orientation.w = 1;
    }
}

bool readParameters(ros::NodeHandle node_handle_)
{
    ROS_INFO_STREAM("Loading parameters.....");
    if (!node_handle_.getParam("/destination_generator/test_world", test_world))
        ROS_WARN_STREAM("Parameter test_world not set. Using default setting: " << test_world);
    if (!node_handle_.getParam("/destination_generator/publish_interval", publish_interval))
        ROS_WARN_STREAM("Parameter publish_interval not set. Using default setting: " << publish_interval);
    if (!node_handle_.getParam("/destination_generator/end_point", end_point))
        ROS_WARN_STREAM("Parameter end_point not set. Using default setting: " << end_point);
    if (!(test_world == "test" || test_world == "diamond" || test_world == "museum" || test_world == "hospital" || test_world == "pomdp_test"))
    {
        ROS_ERROR_STREAM("The selected test_world is not supported, please refer to the options in config file!");
        return false;
    }
    ROS_INFO_STREAM("Complete loading parameters.");
    return true;
}

float calDistance(float x1, float y1, float x2, float y2)
{
    return sqrt(powf((x1 - x2), 2) + powf((y1 - y2), 2));
}

void timerCallback(const ros::TimerEvent& event)
{
    if (end_point == "start")
    {
        global_goal.goal.target_pose.pose = initial_pose;
    }
    else if (end_point == "mid")
    {
        global_goal.goal.target_pose.pose = mid_pose;
    }
    else if (end_point == "end")
    {
        global_goal.goal.target_pose.pose = goal_pose;
    }
    goal_publisher.publish(global_goal);
}

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped &msg_amcl)
{
    // float dist2initial = calDistance(initial_pose.position.x, initial_pose.position.y, msg_amcl.pose.pose.position.x, msg_amcl.pose.pose.position.y);
    // float dist2goal = calDistance(goal_pose.position.x, goal_pose.position.y, msg_amcl.pose.pose.position.x, msg_amcl.pose.pose.position.y);
    // if (dist2initial > dist2goal && dist2goal < 3)
    // {
    //     agent_forward = false;
    // }
    // else
    // {
    //     agent_forward = true;
    // }
    // ROS_INFO_STREAM("dist2initial " << dist2initial);
    // ROS_INFO_STREAM("dist2goal " << dist2goal);
    // ROS_INFO_STREAM("agent_forward " << agent_forward);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "destination_generator");
    ros::NodeHandle node_handle;

    global_goal.goal.target_pose.header.frame_id = "map";
    publish_interval = 0.2;
    test_world = "test";
    agent_forward = true;

    if (!readParameters(node_handle))
    {
        ROS_ERROR_STREAM("Could not load parameters.");
        ros::requestShutdown();
    }
    initializeGoal();

    amcl_subscriber = node_handle.subscribe("/amcl_pose", 1, amclCallback);

    goal_publisher = node_handle.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1, true);

    ros::Timer pub_timer = node_handle.createTimer(ros::Duration(publish_interval), timerCallback);

    std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();

    std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();

    float time_diff = (current_time - start_time).count() / 1000000000.0;

    while(time_diff <= 1.0)
    {
        ros::spinOnce();
        current_time = std::chrono::system_clock::now();
        time_diff = (current_time - start_time).count() / 1000000000.0;
    }
    
    ros::shutdown();

    return 0;
}