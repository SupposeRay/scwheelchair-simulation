// ROS
#include <ros/ros.h>
// tf
#include <tf/transform_datatypes.h>
// Messages
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// C++
#include <math.h>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace trajectory_generator
{
    class TrajectoryGeneratorNode
    {
    public:
        // Constructor
        TrajectoryGeneratorNode(ros::NodeHandle &node_handle);
        // Destructor
        virtual ~TrajectoryGeneratorNode();

        // Public Member Functions


        // Public Member Attributes
        // PI
        double PI = 3.14159265358;
        // user command
        geometry_msgs::Twist user_cmd;
        // odometry
        nav_msgs::Odometry agent_odom;
        // costmap
        nav_msgs::OccupancyGrid local_costmap;
        // bool value to check if the specific msg is received
        bool cmd_receive = false;
        bool odom_receive = false;
        bool amcl_receive = false;
        bool cost_receive = false;
        bool map_receive = false;
        // publish frequency
        double publish_frequency_;
        // linear and angular accelerations
        float linear_acclrt, angular_acclrt;
        // time interval between 2 successive points on the path
        float time_interval;
        // number of points on the path
        int path_length;

    private:
        // Private Member Functions
        bool readParameters();
        void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg_cmd);
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg_map);
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom);
        void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg_amcl);
        void costCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg_cost);
        void timerCallback(const ros::TimerEvent&);
        void cvtMapToGradImg(nav_msgs::OccupancyGrid map_);
        void generateTrajectory();
        void publishResults();

        // Private Member Attributes
        // final path
        nav_msgs::Path agent_trajectory;
        // final goal
        geometry_msgs::PoseStamped agent_goal;
        // gradient image along X axis and Y axis
        cv::Mat gradimg_x, gradimg_y;
        // linear and angular velocity from user command
        float v_cmd = 0;
        float w_cmd = 0;
        // current position of agent from odometry
        float x_agent, y_agent, z_agent; 
        // current velocities from odometry
        float v_agent, w_agent;
        // map size
        int map_cols, map_rows;
        // map resolution
        float map_resolution;
        // origin of costmap with respect to current map frame
        float origin_x, origin_y, origin_z;
        // ROS nodehandle
        ros::NodeHandle &node_handle_;
        
        // ROS subscribers and publishers
        ros::Subscriber cmd_subscriber_;
        ros::Subscriber map_subscriber_;
        ros::Subscriber odom_subscriber_;
        ros::Subscriber amcl_subscriber_;
        ros::Subscriber cost_subscriber_;
        ros::Publisher trajectory_publisher_;
        ros::Publisher goal_publisher_;
        ros::Timer timer_;
    };
}