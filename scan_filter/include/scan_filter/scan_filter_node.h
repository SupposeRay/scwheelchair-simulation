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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
// C++
#include <math.h>
// Package
#include <scat_libs/base_utils.h>

namespace scan_filter
{
class ScanFilterNode
{
public:
    // Constructor
    ScanFilterNode(ros::NodeHandle &node_handle);
    // Destructor
    virtual ~ScanFilterNode();
    // Public Member Functions

private:
    //// Node methods ////
    bool readParameters();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_scan);
    void timerCallback(const ros::TimerEvent &);
    void publishResults();

    //// Algorithm methods ////
    void filterLaserScan();

    //// Parameters ////
    // publish frequency
    double publish_period_ = 0.1;
    // lidar
    sensor_msgs::LaserScan scan_;
    // bool value to check if the specific msg is received
    bool scan_received_ = false;
    // limits
    double range_max_ = 10.0;
    double range_min_ = 0.1;
    double angle_min_ = -3.0;
    double angle_max_ = 3.0;
    
    //// ROS objects ////
    ros::NodeHandle &node_handle_;
    ros::Subscriber scan_subscriber_;
    ros::Publisher scan_publisher_;
};
} // namespace scan_filter