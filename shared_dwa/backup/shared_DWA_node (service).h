// ROS
#include <ros/ros.h>
// tf
#include <tf/transform_datatypes.h>
// Service
#include "shared_dwa/doorway_srv.h"
// Messages
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// Visualization
#include <visualization_msgs/Marker.h>
// C++
#include <math.h>
#include <string>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace shared_DWA
{
    class shared_DWANode
    {
    public:
        // Constructor
        shared_DWANode(ros::NodeHandle &node_handle);
        // Destructor
        virtual ~shared_DWANode();
        // Public Member Functions

        // Public Memeber Attributes

    private:
        // Private Member Functions
        bool readParameters();
        // callback for lidar scan data
        void scanCallback(const sensor_msgs::LaserScan &msg_scan);
        // callback for odometry data
        void odomCallback(const nav_msgs::Odometry &msg_odom);
        // callback for user command
        void cmdCallback(const geometry_msgs::Twist &msg_cmd);
        // callback for time
        void timerCallback(const ros::TimerEvent&);
        // publish final result
        void publishResults();
        // generate dynamic window
        void dynamicWindow();
        // calculate distance to collision
        float calDist2Collision(float v_dw, float w_dw, float sample_interval, float sample_time, float& min_dist2goal, float& angle2goal);
        // collision check
        bool checkCollision(float x_check, float y_check, float theta_check, std::string footprint_mode);
        // some calculation functions
        float calDistance(float x1, float y1, float x2, float y2);
        float calDotproduct(cv::Vec2f v1, cv::Vec2f v2);
        float calHeading(float v_current, float w_current, float v_command, float w_command);
        // select the optimal velocity from dynamic window
        void selectVelocity();
        // generate goal for doorway traversal
        void generateGoal();
        // the cost of different paths to the goal
        cv::Mat calPathcost(cv::Mat dist_cost, cv::Mat angle_cost);

        // Private Member Attributes
        // PI
        double PI = 3.14159265358;
        // publish frequency
        float publish_interval = 0.2;
        // lidar
        sensor_msgs::LaserScan lidar_scan;
        // bool value to check if the specific msg is received
        bool scan_receive = false;
        bool odom_receive = false;
        bool cmd_receive = false;
        // linear and angular acceleration 
        float v_acclrt = 0.2, w_acclrt = 0.523;   // m/s^2 and rad/s^2
        // linear and angular deceleration 
        float v_dclrt = -0.2, w_dclrt = -0.523;   // m/s^2 and rad/s^2
        // sample interval of DWA
        float sample_interval = 0.5;    // s
        // samole time of DWA
        float sample_time = 5;          // s
        // sample number
        int sample_number = 1;
        // samole size of linear and angular velocity space
        int v_sample = 10, w_sample = 10;
        // current max linear and angular velocity
        float v_max, v_min, w_max;  // m/s and rad/s
        // max physical linear and angular velocity of agent
        float v_max_robot = 1.0, v_min_robot = -1.0, w_max_robot = 1.5;
        // sample in dynamic window
        float v_dw = 0, w_dw = 0;
        // agent direction
        bool agent_forward = true;
        // cost function window
        cv::Mat cost_window;
        // guidance cost function window
        cv::Mat path_cost_window;
        // distance cost function window
        cv::Mat dist_cost_window;
        // angle cost function window
        cv::Mat angle_cost_window;
        // dwa cost window
        cv::Mat heading_window;
        cv::Mat clearance_window;
        cv::Mat velocity_window;
        // dynamic window
        cv::Mat dynamic_window;
        // bool value to check if dynamic window is generated
        bool window_generate = false;
        // initialize the weight of heading and velocity
        float weight_heading = 0.7;
        float weight_velocity = 0.3;
        // danger index
        float danger_ = 0; 
        // base_frame_id and fixed_frame_id
        std::string base_frame_id = "base_footprint", fixed_frame_id = "map";
        // footprint mode
        std::string footprint_mode = "radius";
        // if footprint mode is radius
        float r_collision = 0.5;
        // if footprint mode is rectangle
        std::vector<cv::Point2f> rectangle_point;
        // ros time
        ros::Time t_now;
        // trajectory of each sample
        // nav_msgs::Path dwa_path;
        // makers to show in Rviz
        visualization_msgs::Marker candidate_samples;
        visualization_msgs::Marker final_line;
        visualization_msgs::Marker user_line;
        visualization_msgs::Marker doorway_marker;
        // twist msg to be published
        geometry_msgs::Twist dwa_twist;
        // current velocities from odometry
        float v_agent = 0, w_agent = 0;
        // velocities from command
        float v_cmd = 0, w_cmd = 0;
        // ROS nodehandle
        ros::NodeHandle &node_handle_;
        // ROS subscribers and publishers 
        ros::Subscriber scan_subscriber_;
        ros::Subscriber odom_subscriber_;
        ros::Subscriber cmd_subscriber_;
        ros::Publisher vel_publisher_;
        // ros::Publisher path_publisher_;
        ros::Publisher cddt_publisher_;
        ros::Publisher line_publisher_;
        ros::Publisher ucmd_publisher_;
        ros::Publisher goal_publisher_;
        ros::Timer timer_;
        // ROS service client
        ros::ServiceClient doorway_client_;
        // instantiate service
        shared_dwa::doorway_srv detection_srv;
        // doorway goal
        geometry_msgs::Pose doorway_goal;
        // bool value to check if a door is detected
        bool doorway_found = false;
        // doorway goal quaternion
        tf::Quaternion goal_quat;
        // doorway goal theta
        float goal_yaw = 0;
        // weight allocated to distance when computing the cost of closeness
        float weight_distance = 0.8;
        // weight allocated to user command and goal alignment when using goal guidance
        float weight_cmd = 0.1, weight_goal = 0.9;
    };
}