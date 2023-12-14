// ROS
#include <ros/ros.h>
// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// Messages
#include <std_msgs/UInt32.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
// Custom Messages
#include <path_belief_update/WaypointDistribution.h>
#include <voronoi_msgs_and_types/PathList.h>
// Visualization
// #include <visualization_msgs/Marker.h>
// C++
#include <math.h>
#include <string>
#include <numeric>
#include <algorithm>
// #include <functional>

namespace belief_update
{
    class belief_updateNode
    {
    public:
        // Constructor
        belief_updateNode(ros::NodeHandle &node_handle);
        // Destructor
        virtual ~belief_updateNode();
        // Public Member Functions

        // Public Memeber Attributes

    private:
        // Private Member Functions
        bool readParameters();
        // callback for odometry data
        void odomCallback(const nav_msgs::Odometry &msg_odom);
        // callback for user input
        void pointCallback(const geometry_msgs::Point &msg_point);
        void twistCallback(const geometry_msgs::Twist &msg_twist);
        // callback for goal list from path
        void pathCallback(const voronoi_msgs_and_types::PathList &msg_path);
        // callback for topics publishing timer
        void publishTimerCallback(const ros::TimerEvent&);
        // callback for belief updating timer
        void beliefTimerCallback(const ros::TimerEvent&);
        // publish final result
        void PublishResults();
        // tranform the path list from map to local
        void pathToLocal(voronoi_msgs_and_types::PathList &msg_path);
        // redistribute the probabilities
        std::vector<float> redistributeProb(voronoi_msgs_and_types::PathList old_path, voronoi_msgs_and_types::PathList new_path, std::vector<int> old_index, std::vector<int> new_index, std::vector<float> prob_path);
        // generate short-term goals from paths for belief updating
        void generateGoal();
        // find a goal along a path
        geometry_msgs::Pose findGoal(const nav_msgs::Path &msg_path);
        // compute the translational state value V of each goal given current state (position)
        // float calTranslationStateValue(geometry_msgs::Pose goal_pose);
        // compute the rotational state value V of each goal given current state (position)
        // float calRotationStateValue(geometry_msgs::Pose goal_pose);
        // compute the translational state-action value Q of each goal given current state (position) and action (u,v)
        // float calTranslationStateActionValue(float v_action, float w_action, geometry_msgs::Pose goal_pose);
        // compute the rotational state-action value Q of each goal given current state (position) and action (u,v)
        // float calRotationStateActionValue(float v_action, float w_action, geometry_msgs::Pose goal_pose);
        float calRotationValue(float x_input, float y_input, geometry_msgs::Pose goal_pose);
        // compute the translational action cost
        // float calTranslationActionCost(float dis_next);
        // compute the rotational action cost
        float calRotationActionCost(float angle_next);
        // update the belief space
        void updateGoalPrediction();
        // some calculation functions
        float calDistance(float x1, float y1, float x2, float y2);

        /**
         * Get agent pose using TF
         **/
        // geometry_msgs::PoseStamped getGlobalAgentPose();

        std::vector<float> softMax(std::vector<float> belief_vector);
        std::vector<float> clipBelief(std::vector<float> belief_vector);
        std::vector<float> normalize(std::vector<float> belief_vector);

        // Private Member Attributes
        // PI
        double PI = 3.14159265358;
        // ROS nodehandle
        ros::NodeHandle &node_handle_;
        // ROS subscribers and publishers 
        ros::Subscriber odom_subscriber_;
        ros::Subscriber path_subscriber_;
        ros::Subscriber input_subscriber_;

        ros::Publisher goal_publisher_;
        ros::Publisher path_publisher_;
        ros::Timer belief_timer_;
        ros::Timer publish_timer_;
        // the interval of updating belief with user action
        float update_interval = 0.5;
        // the interval of publishing topics
        float publish_interval = 0.1;
        // a bool value to trigger the action update timer
        bool action_update = false;
        // the belief space
        std::vector<float> belief_goal;
        // obtained local goal_list
        geometry_msgs::PoseArray goal_list;
        // signal obtained from published paths
        voronoi_msgs_and_types::PathList path_list;
        // previous published paths to check the change
        voronoi_msgs_and_types::PathList pre_path_list;
        // the current waypoints and their probability distribution
        path_belief_update::WaypointDistribution waypoint_belief;
        // bool value to check if a specific msg is received
        bool odom_receive = false;
        bool input_receive = false;
        bool path_receive = false;
        // current velocities from odometry
        float v_agent = 0, w_agent = 0;
        // // current agent pose
        // geometry_msgs::Pose agent_pose;
        // the user input datatype
        std::string input_datatype = "point";
        // signals from user input
        float x_cmd = 0, y_cmd = 0;
        // the max values of x and y inputs to do the normalization
        float x_max = 1.0, y_max = 1.0;
        // the discount factor for state-action value
        float discount_factor = 1.5;
        // the rate of updating probability distribution
        float rate_factor = 3.0;
        // the angle threshold for computing cost function
        float angle_threshold = 90;
        // the constant value added to the rotation cost function
        float angle_const_cost = 1;
        // frame_ids
        std::string base_frame_id = "base_link", path_frame_id = "map", odom_frame_id = "odom";
        // how long to block before failing for tf transformation
        float tf_buffer_timeout = 0.5;
        // the threshold of distance to generate the next waypoint
        float waypoint_dist = 1;
        // the factor to adjust the rate of doing softmax
        float temp_softmax = 2;
        // the thresholds for clipping the probability
        float upper_bound = 0.9, lower_bound = 0.1;
        // the most likely goal index
        // int goal_index = 0;
        // samole time of DWA
        float sample_time = 3;
        // the increment of distance to generate the next waypoint
        float waypoint_increment = 1.0;
        // previous goal index
        // int pre_goal_index = -1;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;
    };

    class seq_finder
    {
    public:
        seq_finder(const int seq)
        :path_sequence(seq){}

        bool operator ()(const std::vector<nav_msgs::Path>::value_type &path)
        {
            return path.header.seq == path_sequence;
        }
    
    private:
        int path_sequence;                    
    };
}