// ROS
#include <ros/ros.h>
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
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include <obstacle_detector/Obstacles.h>
// Custom Messages
#include <path_belief_update/WaypointDistribution.h>
// Visualization
#include <visualization_msgs/Marker.h>
// C++
#include <math.h>
#include <string>
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

        // Public Member Functions

        // Public Memeber Attributes

    private:
        // Private Member Functions
        bool readParameters();

        /**
         * Callback for dynamic and static obstacles from obstacle tracking node
         **/
        void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr &msg);

        // callback for lidar scan data
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_scan);
        // callback for odometry data
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom);
        // callback for amcl data
        // void amclCallback(const geometry_msgs::PoseWithCovarianceStamped &msg_amcl);
        // callback for user command
        void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg_cmd);
        // callback for goal data
        void goalCallback(const path_belief_update::WaypointDistribution::ConstPtr &msg_goal);
        // callback for status
        void statusCallback(const std_msgs::Bool::ConstPtr &msg_status);

        /**
         * Forward simulate all obstacles that were received by tracker
         * @param obstacles obstacles received from tracker node
         * @param sample_number how many time samples to forward simulate into the future
         * @param sample_interval interval between each sample
         * @return returns a 2D vector of obstacles. vec[i][n] means position of obstacle n at time i into the future
         **/
        std::vector<std::vector<obstacle_detector::CircleObstacle>> forwardSimulateObstacles(const std::vector<obstacle_detector::CircleObstacle>& obstacles, int sample_number, double sample_interval);

        /**
         * Check for collision using vector representation of obstacles (line segmenets and circles)
         * @param x_check x position of robot to check, in robot frame
         * @param y_check y position of robot to check, in robot frame
         * @param theta_check orientation of robot to check, affects total footprint if footprint mode is rectangle
         * @param footprint_mode footprint mode used to check for collision
         * @param current_sample current sample number along the forward simulated trajectory
         * @return true if collision happened
         **/
        bool checkDynamicCollision(float x_check, float y_check, float theta_check,const std::string& footprint_mode, int current_sample);

        /**
         * Check for collision using raw laser sensor readings
         * @param x_check x position of robot to check, in robot frame
         * @param y_check y position of robot to check, in robot frame
         * @param theta_check orientation of robot to check, affects total footprint if footprint mode is rectangle
         * @param footprint_mode footprint mode used to check for collision
         * @return true if collision happened
         **/
        bool checkCollision(float x_check, float y_check, float theta_check,const std::string& footprint_mode);

        /**
         * Timer callback which generates dynamic window and publishes output velocity if joystick command is received
         **/
        void timerCallback(const ros::TimerEvent &);

        // publish final result
        void publishResults();
        // generate dynamic window
        bool dynamicWindow();
        // calculate distance to collision
        float calDist2Collision(float v_dw, float w_dw, std::vector<float> &min_dist2goal, std::vector<float> &angle2goal, std::vector<geometry_msgs::Point> &candidate_points);

        /**
         * Calculate euclidean distance 
         **/
        float calDistance(float x1, float y1, float x2, float y2);

        /**
         * Calculate squared euclidean distance
         **/
        float squaredDistance(float x1, float y1, float x2, float y2);

        /**
         * Calculates the shortest distance between a point and a line segment
         * @param v1 first point of line segment
         * @param v2 last point of line segment
         * @param p point to get distance to from line segment
         * @return returns the shortest distance between point and line segment
         **/
        float minDistPointToSegment(const cv::Vec2f& v1, const cv::Vec2f& v2, const cv::Vec2f& p);

        /**
         * Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr' 
         * @param p first point of line pr
         * @param q point to check if on line pr
         * @param r last point of line pr
         * @return returns true if q lies on segment pr
         **/
        bool onSegment(cv::Point2f p, cv::Point2f q, cv::Point2f r);
        
        /**
         * To find orientation of ordered triplet (p, q, r). https://www.geeksforgeeks.org/orientation-3-ordered-points/
         * @param p point 1
         * @param q point 2
         * @param r point 3
         * @return returns 0 if colinear, 1 if clockwise (order pqr), 2 if counter-clockwise
         **/
        int orientation(cv::Point2f p, cv::Point2f q, cv::Point2f r);

        /**
         * Checks if line segment p1q1 intersects p2q2
         * https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
         * @param p1 point 1 of line segment 1
         * @param q1 point 2 of line segment 1
         * @param p2 point 1 of line segment 2
         * @param q2 point 2 of line segment 2
         * @return returns true if the line segments intersect
         **/
        bool doIntersect(cv::Point2f p1, cv::Point2f q1, cv::Point2f p2, cv::Point2f q2);

        /**
         * Calculate dot product of 2 vectors
         **/
        float calDotproduct(cv::Vec2f v1, cv::Vec2f v2);

        /**
         * Gets the average of sum of squared distances of all LIDAR points from center of robot that lies in robot's footprint
         * High computational load
         **/
        void calcObsInterference(float x_check, float y_check, float theta_check, const std::string &footprint_mode, float &sum, int &num_points);

        void calcDynamicObsInterference(float x_check, float y_check, float theta_check, const std::string &footprint_mode, int current_sample, float &sum, int &num_points);

        float calHeading(float v_current, float w_current, float v_command, float w_command);
        // select the optimal velocity from dynamic window
        void selectVelocity();
        // generate goal for dwa
        bool generateGoal(const path_belief_update::WaypointDistribution &msg_waypoint);
        // the cost of different paths to the goal
        cv::Mat calPathcost(cv::Mat dist_cost, cv::Mat angle_cost);

        // Private Member Attributes
        // publish frequency
        float publish_interval = 0.2;

        std::vector<geometry_msgs::Point> lidar_points;
        // bool value to check if the specific msg is received
        bool inside_inflation = false;
        bool cmd_receive = false;
        bool goal_received = false;
        bool global_received = false;

        // linear and angular acceleration
        float v_acclrt = 0.2, w_acclrt = 0.523; // m/s^2 and rad/s^2

        // sample interval of DWA
        float sample_interval = 0.5; // s

        // samole time of DWA
        float sample_time = 5; // s

        // sample number
        int sample_number = 1;

        // samole size of linear and angular velocity space
        int v_sample = 10, w_sample = 10;

        // current max linear and angular velocity
        float v_max, w_max; // m/s and rad/s

        //Maximum linear velocity when obstacle is in robot footprint
        float v_max_collision = 0.15;

        //Maximum angular velocity when obstacle is in robot footprint
        float w_max_collision = 0.15;

        // max physical linear and angular velocity of agent
        float v_max_robot = 1.0, v_min_robot = -1.0, w_max_robot = 1.5;

        bool enable_visualization = true;

        float dynamic_window_time_interval = 0.5;

        // agent direction
        bool agent_forward = true;

        // waypoints distribution
        path_belief_update::WaypointDistribution waypoint_belief;

        // probability distribution of waypoints
        std::vector<float> belief_goal;

        // index of most likely goal
        int max_index;

        // cost function window
        cv::Mat cost_window;

        std::vector<cv::Mat> matList;

        // guidance cost function window
        std::vector<cv::Mat> path_cost_window;

        // distance cost function window
        std::vector<cv::Mat> dist_cost_window;

        // angle cost function window
        std::vector<cv::Mat> angle_cost_window;

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
        // whether to use the most likely goal or expected cost
        bool use_expected_cost = true;
        // danger index
        float danger_ = 0;

        // base_frame_id and fixed_frame_id
        std::string base_frame_id = "base_link", fixed_frame_id = "map";

        // footprint mode
        std::string footprint_mode = "radius";

        // if footprint mode is radius
        float r_collision = 0.5;

        // if footprint mode is rectangle
        std::vector<cv::Point2f> rectangle_point;

        // markers to show in Rviz
        visualization_msgs::Marker candidate_samples;
        visualization_msgs::Marker final_line;
        visualization_msgs::Marker user_line;
        visualization_msgs::Marker robot_line;
        visualization_msgs::Marker waypoint_viz;

        // twist msg to be published
        geometry_msgs::Twist dwa_twist;

        // current velocities from odometry
        float v_agent = 0, w_agent = 0;

        // // current orientation from odometry
        // tf2::Quaternion agent_quat;

        // velocities from command
        float v_cmd = 0, w_cmd = 0;

        // whether to trigger the guidance mode to guide the shared control
        std::string guidance_mode = "disable";

        // the goal for dwa
        // geometry_msgs::PoseStamped dwa_goal;

        //minimum clearance threshold for path to be viable
        float min_clearance_threshold = 0.1;

        // the goal data
        geometry_msgs::Pose goal_data;

        // the global goal in base_link frame
        geometry_msgs::Pose global_goal;

        /**
         * Minimum distance to global goal, calculated when forward simulating trajectories
         **/ 
        float min_dist2global = 100;

        // bool value to check if a goal is detected
        bool goal_found = false;
        // the goal quaternion
        tf2::Quaternion goal_quat;

        // the orientation of the goal
        std::vector<float> goal_yaw;
        // the distance to the goal
        std::vector<float> dist2goal;

        geometry_msgs::PoseArray goal_list;

        // weight allocated to distance when computing the cost of closeness
        float weight_distance = 0.8;

        // weight allocated to user command and goal alignment when using goal guidance
        float weight_cmd = 0.1, weight_goal = 0.9;

        // the lower bound of weight assigned to cmd
        float weight_cmd_lb = 0.1;

        // whether to enable a dynamic weight assignment to goal and user command
        bool enable_dynamic_weight = false;

        // the gamma transformation factors to scale the cost windows for goal and command
        float gamma_cmd = 1.0, gamma_goal = 1.0;

        // how long to block before failing for tf transformation
        float tf_buffer_timeout = 0.3;

        /**
         * Whether or not to use dynamic obstacles obtained from obstacle detector
         **/
        bool use_dynamic_obstacles = false;

        /**
         * Transform to store static transform between lidar and base_link if the frames are not identical
         **/
        geometry_msgs::TransformStamped lidar2baseTransform;

        /**
         * Variable to store all line segment obstacles received from tracker node
         **/
        std::vector<obstacle_detector::SegmentObstacle> segment_obstacles;

        /**
         * Vector to store all circle obstacles before forward simulating
         **/
        std::vector<obstacle_detector::CircleObstacle> circle_obstacles;

        /** 
         * 2D array to store all forward simulated dynamic obstacles, for use in collision checking
         * vec[i][n] means position of obstacle n at time i into the future
         **/
        std::vector<std::vector<obstacle_detector::CircleObstacle>> forward_sim_obs;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        // ROS nodehandle
        ros::NodeHandle &node_handle_;
        // ROS subscribers and publishers
        ros::Subscriber scan_subscriber_;
        ros::Subscriber odom_subscriber_;
        ros::Subscriber obstacles_subscriber_;
        // ros::Subscriber amcl_subscriber_;
        ros::Subscriber cmd_subscriber_;
        ros::Subscriber goal_subscriber_;
        ros::Subscriber status_subscriber_;

        ros::Publisher vel_publisher_;
        ros::Publisher cddt_publisher_;
        ros::Publisher line_publisher_;
        ros::Publisher ucmd_publisher_;
        ros::Publisher rcmd_publisher_;
        ros::Publisher wypt_publisher_;
        ros::Publisher goal_publisher_;
        ros::Publisher cancel_publisher_;
        ros::Timer timer_;
    };
}