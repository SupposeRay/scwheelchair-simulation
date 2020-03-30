#include "scan_filter/scan_filter_node.h"
namespace scan_filter
{

// constructor
ScanFilterNode::ScanFilterNode(ros::NodeHandle &node_handle)
    : node_handle_(node_handle)
{
    if (!readParameters())
    {
        ROS_ERROR("Could not load parameters scan filter.");
        ros::requestShutdown();
    }

    // subscribers & publishers
    scan_subscriber_ = node_handle_.subscribe("/scan", 1, &ScanFilterNode::scanCallback, this);
    scan_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("/scan/filtered", 1);
}

// destructor
ScanFilterNode::~ScanFilterNode()
{
}

bool ScanFilterNode::readParameters()
{
    if (!node_handle_.getParam("range_max", range_max_))
        ROS_WARN_STREAM("Parameter range_max not set for scan_filter. Using default setting: " << range_max_);
    if (!node_handle_.getParam("range_min", range_min_))
        ROS_WARN_STREAM("Parameter range_min not set for scan_filter. Using default setting: " << range_min_);
    if (!node_handle_.getParam("angle_max", angle_max_))
        ROS_WARN_STREAM("Parameter angle_max not set for scan_filter. Using default setting: " << angle_max_);
    if (!node_handle_.getParam("angle_min", angle_min_))
        ROS_WARN_STREAM("Parameter angle_min not set for scan_filter. Using default setting: " << angle_min_);

    if (!(
            (range_max_ > range_min_) &&
            (angle_max_ > angle_min_) &&
            (range_max_ < 25.0) &&
            (range_min_ > 0.1) &&
            (angle_min_ > -M_PI) &&
            (angle_max_ < M_PI)))
    {
        ROS_WARN("Invalid parameter values.");
        return false;
    }

    ROS_DEBUG("Parameters  correct.");
    return true;
}

void ScanFilterNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_scan)
{
    scan_.header = msg_scan->header;
    scan_.angle_min = msg_scan->angle_min;
    scan_.angle_max = msg_scan->angle_max;
    scan_.angle_increment = msg_scan->angle_increment;
    scan_.time_increment = msg_scan->time_increment;
    scan_.scan_time = msg_scan->scan_time;
    scan_.range_min = msg_scan->range_min;
    scan_.range_max = msg_scan->range_max;
    scan_.ranges = msg_scan->ranges;
    scan_.intensities = msg_scan->intensities;
    scan_received_ = true;
    filterLaserScan();
    publishResults();
}

void ScanFilterNode::publishResults()
{
    if (scan_received_)
    {
        scan_publisher_.publish(scan_);
    }
    else
        ROS_ERROR_STREAM("Error! Scan filter received no LaserScan msg.");
}

void ScanFilterNode::filterLaserScan()
{
    for (int i = 0; i < scan_.ranges.size(); i++)
    {
        // Retrieve measurement angle in robot frame of reference.
        double angle = base_utils::shiftAngle(scan_.angle_min + (double)i * scan_.angle_increment + M_PI);
        // If measurement does not satisfy all range settings, set to -1.
        if ((scan_.ranges[i] > range_max_) || (scan_.ranges[i] < range_min_) ||
            (angle < angle_min_) || (angle > angle_max_))
            scan_.ranges[i] = -1;
    }
}

} // namespace scan_filter