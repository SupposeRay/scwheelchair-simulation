// ROS
#include <ros/ros.h>
// Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
// C++
#include <math.h>
#include <string>
#include <deque>
#include <numeric>
// #include <std_msgs/Float32MultiArray.h>

namespace input_converter
{
    class input_converterNode
    {
    public:
        //Constructor
        input_converterNode(ros::NodeHandle &node_handle);
        // Destructor
        virtual ~input_converterNode();
        // Public Member Functions

        // Public Memeber Attributes

    private:
        // Private Member Functions

        // read parameters from config file
        bool readParameters();
        // callback for joystick input
        void joystickCallback(const geometry_msgs::Point &msg_joystick);
        // callback for time
        void timerCallback(const ros::TimerEvent&);

        // publish final result
        void publishResults();

        // filter
        void meanFilter(geometry_msgs::Point &msg_input);

        // Private Member Attributes

        // joystick message
        geometry_msgs::Point input_joystick;
        // ros time
        ros::Time t_now;
        // twist msg to be published
        geometry_msgs::Twist cmd_twist;
        geometry_msgs::Twist old_cmd_twist;

        // ROS nodehandle
        ros::NodeHandle &node_handle_;
        // ROS subscribers and publishers 
        ros::Subscriber joystick_subscriber_;
        ros::Subscriber keyboard_subscriber_;
        ros::Publisher vel_publisher_;
        ros::Timer timer_;

        // bool value to check if the specific msg is received
        bool joystick_receive = false;
        bool add_filter = false;

        // default config parameters
        // publish frequency
        float publish_interval = 0.2;
        // filter
        std::string filter_type = "None";

        // mean filter
        std::deque<float> buffer_x_;
        std::deque<float> buffer_y_;
        int buffer_size_ = 5;
        float average_x_ = 0;
        float average_y_ = 0;

        // parameters to scale the maximum linear and angular velocities
        float linear_scale = 0.5;
        float angular_scale = 0.5;
	    // max linear and angular accelerations
        float linear_acc = 1.0;
        float angular_acc = 1.0;
    };
}
