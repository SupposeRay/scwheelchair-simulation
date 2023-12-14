// ROS
#include <ros/ros.h>
// #include <signal.h>
// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
// #include <tf2/buffer_core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// actionlib
#include <actionlib/client/simple_action_client.h>
// ROS action
#include "shared_dwa/doorway_detectionAction.h"
// Messages
#include <std_msgs/Float32.h>
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
#include <std_msgs/Float32MultiArray.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/PolygonStamped.h>
// Visualization
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// C++
#include <math.h>
#include <string>
#include <iomanip>
#include <sstream>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

typedef actionlib::SimpleActionClient<shared_dwa::doorway_detectionAction> Doorway_Client;

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
        // callback for amcl data
        // void amclCallback(const geometry_msgs::PoseWithCovarianceStamped &msg_amcl);
        // callback for user command
        void cmdCallback(const geometry_msgs::Point &msg_cmd);
        // callback for goal data
        void goalCallback(const geometry_msgs::PoseArray &msg_goal);
        // callback for the global goal
        void globalCallback(const geometry_msgs::Pose &msg_global);
        // check if the final destination is reached
        void checkDestination();
        // callback for algo_timer
        void algotimerCallback(const ros::TimerEvent&);
        // callback for pub_timer
        void pubtimerCallback(const ros::TimerEvent&);
        // publish final result
        void publishResults();
        // generate dynamic window
        bool dynamicWindow();
        // calculate distance to collision
        float calDist2Collision(float v_dw, float w_dw, float& min_dist2goal, float& angle2goal, std::vector<geometry_msgs::Point>& candidate_points);
        // collision check
        bool checkCollision(float x_check, float y_check, float theta_check, std::string footprint_mode);
        // some calculation functions
        float calDistance(float x1, float y1, float x2, float y2);
        float calDotproduct(cv::Vec2f v1, cv::Vec2f v2);
        float calHeading(float v_current, float w_current, float v_command, float w_command);
        // select the optimal velocity from dynamic window
        void selectVelocity();
        // generate goal for dwa
        bool generateGoal();
        // the cost of different paths to the goal
        cv::Mat calPathcost(cv::Mat dist_cost, cv::Mat angle_cost);

        // Private Member Attributes
        // publish frequency
        float publish_interval = 0.2;
        float algorithm_interval = 0.2;
        
        std::vector<geometry_msgs::Point> lidar_points;
        // bool value to check if the specific msg is received
        bool cmd_receive = false;
        bool goal_receive = false;
        bool global_receive = false;

        // linear and angular acceleration 
        float v_acclrt = 0.2, w_acclrt = 0.523;   // m/s^2 and rad/s^2

        // samole time of DWA
        float sample_time = 5;          // s

        /**
         * Distance between each sample when forward simulating trajectories
         **/
        float sample_distance = 0.1;

        // samole size of linear and angular velocity space
        int v_sample = 10, w_sample = 10;

        // current max linear and angular velocity
        float v_max, v_min, w_max;  // m/s and rad/s

        // max physical linear and angular velocity of agent
        float v_max_robot = 1.0, v_min_robot = -1.0, w_max_robot = 1.5;

        bool enable_visualization = true;

        float dynamic_window_time_interval = 0.5;

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

        // initialize the weight of heading and velocity
        float weight_heading = 0.7;
        float weight_velocity = 0.3;

        // danger index
        float danger_ = 0; 

        // base_frame_id and fixed_frame_id
        std::string base_frame_id = "base_link", fixed_frame_id = "map";

        // footprint mode
        std::string footprint_mode = "radius";

        // if footprint mode is radius
        float r_collision = 0.5;

        // if footprint mode is rectangle
        std::vector<geometry_msgs::Point32> rectangle_point;

        // footprint for Rviz
        geometry_msgs::PolygonStamped footprint_polygon;

        // markers to show in Rviz
        visualization_msgs::Marker candidate_samples;
        visualization_msgs::Marker final_line;
        visualization_msgs::Marker udwa_line;
        visualization_msgs::Marker cmd_line;
        visualization_msgs::Marker robot_line;
        // visualization_msgs::Marker waypoint_viz;
        visualization_msgs::Marker waypoint_txt;
        visualization_msgs::MarkerArray waypoint_prob;
        // visualization_msgs::MarkerArray line_namelist;

        // twist msg to be published
        geometry_msgs::Twist dwa_twist;
        geometry_msgs::Twist pub_dwa_twist;
        geometry_msgs::Twist old_dwa_twist;

        // twist msg to be published
        geometry_msgs::Twist pure_dwa_twist;
        // geometry_msgs::Twist pub_pure_dwa_twist;
        // geometry_msgs::Twist old_pure_dwa_twist;

        // current velocities from odometry
        float v_agent = 0, w_agent = 0;

        // // current orientation from odometry
        // tf2::Quaternion agent_quat;

        // velocities from command
        float v_cmd = 0, w_cmd = 0;

        // whether to trigger the guidance mode to guide the shared control
        std::string guidance_mode = "disable";

        // the goal for dwa
        geometry_msgs::PoseStamped dwa_goal;

        //minimum clearance threshold for path to be viable
        float min_clearance_threshold = 0.1;

        // the goal data
        geometry_msgs::PoseArray goal_data;

        std::vector<float> belief_goal;

        // the global goal in base_link frame
        geometry_msgs::Pose global_goal;

        // empty global goal ID
        actionlib_msgs::GoalID dummy_ID;

        // minimum distance to the global goal
        float min_dist2global = 100;

        // bool value to check if a goal is detected
        bool goal_found = false;
        // the goal quaternion
        tf2::Quaternion goal_quat;

        // the orientation of the goal
        float goal_yaw = 0;

        // weight allocated to distance when computing the cost of closeness
        float weight_distance = 0.8;

        // weight allocated to user command and goal alignment when using goal guidance
        float weight_cmd = 0.1, weight_goal = 0.9;
        std_msgs::Float32 final_cmd_weight;

        // the lower bound of weight assigned to cmd
        float weight_cmd_lb = 0.1;

        // whether to enable a dynamic weight assignment to goal and user command
        bool enable_dynamic_weight = false;

        // the gamma transformation factors to scale the cost windows for goal and command
        float gamma_cmd = 1.0, gamma_goal = 1.0;

        // how long to block before failing for tf transformation 
        float tf_buffer_timeout = 0.3;

        // ros::Time prev_time;
        // ros::Time current_time;
        // ros::Duration time_duration;

        geometry_msgs::TransformStamped lidar2baseTransform;

        // ROS nodehandle
        ros::NodeHandle &node_handle_;
        // ROS subscribers and publishers 
        ros::Subscriber scan_subscriber_;
        ros::Subscriber odom_subscriber_;
        // ros::Subscriber amcl_subscriber_;
        ros::Subscriber cmd_subscriber_;
        ros::Subscriber goal_subscriber_;
        ros::Subscriber global_subscriber_;

        ros::Publisher vel_publisher_;
        ros::Publisher dwa_vel_publisher_;
        ros::Publisher cddt_publisher_;
        ros::Publisher line_publisher_;
        ros::Publisher udwa_publisher_;
        ros::Publisher ucmd_publisher_;
        ros::Publisher rdwa_publisher_;
        // ros::Publisher wypt_publisher_;
        ros::Publisher text_publisher_;
        ros::Publisher list_publisher_;
        ros::Publisher weight_publisher_;
        // ros::Publisher goal_publisher_;
        ros::Publisher cancel_publisher_;
        ros::Publisher footprint_publisher_;
        ros::Timer algo_timer_;
        ros::Timer pub_timer_;
        ros::AsyncSpinner *dwa_spinner_;

        ros::Publisher collision_marker_publisher_;
        visualization_msgs::Marker collision_marker_;
    };
}