#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iomanip>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
// #include <geometry_msgs/Point.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgproc.hpp>
// #include <voronoi_msgs_and_types/PathList.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2/transform_datatypes.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <visualization_msgs/Marker.h>

ros::Subscriber raw_subscriber;
ros::Publisher cmd_publisher;
ros::Timer cmd_timer;

std::ifstream data_file;
std::string raw_data_topic = "/raw_data";
std::string published_topic = "/user_joystick";
float linear_scale = 1.0;
float angular_scale = 1.0;
std::string calib_path = "/home/mrca/";
std::string calib_name = "joystick_calibration.txt";

geometry_msgs::Point raw_msg;   // use x & y to store the joystick's raw x & y values, use z to store the raw angle value
bool raw_received = false;
geometry_msgs::Point final_data;    // used to publish the calibrated data
geometry_msgs::Point prev_data;    // used to publish the calibrated data

geometry_msgs::Point front_xtreme;
geometry_msgs::Point back_xtreme;
geometry_msgs::Point left_xtreme;
geometry_msgs::Point right_xtreme;
geometry_msgs::Point central_point;

std::vector<double> x_profile;
std::vector<double> y_profile;
int data_index = 0;
double x_magnitude = 0;
double y_magnitude = 0;

float alpha = 0.18; // low pass filter

void rawCallback(const geometry_msgs::Point::ConstPtr &msg_raw)
{
    raw_msg = *msg_raw;
    raw_received = true;
}

void cmd_timerCallback(const ros::TimerEvent &)
{
    if (raw_received)
    {
        data_index = round(raw_msg.z / 5);
        x_magnitude = x_profile[data_index] - central_point.x;
        y_magnitude = y_profile[data_index] - central_point.y;

        final_data.x = cos(raw_msg.z * M_PI / 180) * raw_msg.x / x_magnitude;
        final_data.y = sin(raw_msg.z * M_PI / 180) * raw_msg.y / y_magnitude;

        final_data.x = (1 - alpha) * prev_data.x + alpha * final_data.x;
        final_data.y = (1 - alpha) * prev_data.y + alpha * final_data.y;

        if (final_data.x > 1)
        {
            final_data.x = 1;
        }
        else if (final_data.x < -1)
        {
            final_data.x = -1;
        }
        // else if (final_data.x < 0.1 && final_data.x > -0.1)
        // {
        //     final_data.x = 0;
        // }

        if (final_data.y > 1)
        {
            final_data.y = 1;
        }
        else if (final_data.y < -1)
        {
            final_data.y = -1;
        }
	    // else if (final_data.y < 0.1 && final_data.y > -0.1)
        // {
        //     final_data.y = 0;
        // }

        // final_data.x = round(final_data.x * 100) / 100;
        // final_data.y = round(final_data.y * 100) / 100;
        final_data.z = 0;

        prev_data = final_data;

        final_data.x = round(final_data.x * linear_scale * 100) / 100;
        final_data.y = round(final_data.y * angular_scale * 100) / 100;

        cmd_publisher.publish(final_data);
        // prev_data = final_data;
    }
    else
    {
        ROS_WARN_STREAM("Raw joystick data not received yet!");
    }
}

int main(int argc, char** argv)
{
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::init(argc, argv, "joystick_control");
    auto node_name = ros::this_node::getName();
    std::cout << node_name << " started." << std::endl;
    ros::NodeHandle node_handle("~");
    node_handle.getParam("raw_data_topic", raw_data_topic);
    node_handle.getParam("calib_path", calib_path);
    node_handle.getParam("calib_name", calib_name);
    node_handle.getParam("published_topic", published_topic);
    node_handle.getParam("linear_scale", linear_scale);
    node_handle.getParam("angular_scale", angular_scale);

    x_profile.resize(72, 0.0);
    y_profile.resize(72, 0.0);

    final_data.x = 0.0;
    final_data.y = 0.0;
    final_data.z = 0.0;
    prev_data = final_data;

    std::cout << "Reading calibration data..." << std::endl;

    data_file.open(calib_path + calib_name);

    if (!data_file.is_open())
    {
        std::cout << "Failed to load the calibration data, plese check if the path is correct and the file exists!" << std::endl;
        ros::shutdown();
        exit(0);
    }

    std::string calib_line;
    std::string delimiter;
    std::string parsed_data;
    size_t parsed_pos = 0;

    // First read 5 extreme points
    std::cout << "Now reading 5 extreme points..." << std::endl;

    // Central point
    std::getline(data_file, calib_line);
    delimiter = ": ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // x point
    delimiter = ", ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        parsed_data = calib_line.substr(0, parsed_pos);
        central_point.x = stod(parsed_data);
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // y point
    delimiter = ", ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        parsed_data = calib_line.substr(0, parsed_pos);
        central_point.y = stod(parsed_data);
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // z point
    central_point.z = stod(calib_line);

    // Frontmost point
    std::getline(data_file, calib_line);
    delimiter = ": ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // x point
    delimiter = ", ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        parsed_data = calib_line.substr(0, parsed_pos);
        front_xtreme.x = stod(parsed_data);
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // y point
    delimiter = ", ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        parsed_data = calib_line.substr(0, parsed_pos);
        front_xtreme.y = stod(parsed_data);
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // z point
    front_xtreme.z = stod(calib_line);

    // Backmost point
    std::getline(data_file, calib_line);
    delimiter = ": ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // x point
    delimiter = ", ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        parsed_data = calib_line.substr(0, parsed_pos);
        back_xtreme.x = stod(parsed_data);
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // y point
    delimiter = ", ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        parsed_data = calib_line.substr(0, parsed_pos);
        back_xtreme.y = stod(parsed_data);
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // z point
    back_xtreme.z = stod(calib_line);

    // Leftmost point
    std::getline(data_file, calib_line);
    delimiter = ": ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // x point
    delimiter = ", ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        parsed_data = calib_line.substr(0, parsed_pos);
        left_xtreme.x = stod(parsed_data);
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // y point
    delimiter = ", ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        parsed_data = calib_line.substr(0, parsed_pos);
        left_xtreme.y = stod(parsed_data);
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // z point
    left_xtreme.z = stod(calib_line);

    // Rightmost point
    std::getline(data_file, calib_line);
    delimiter = ": ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // x point
    delimiter = ", ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        parsed_data = calib_line.substr(0, parsed_pos);
        right_xtreme.x = stod(parsed_data);
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // y point
    delimiter = ", ";
    parsed_pos = calib_line.find(delimiter);
    if (parsed_pos != std::string::npos)
    {
        parsed_data = calib_line.substr(0, parsed_pos);
        right_xtreme.y = stod(parsed_data);
        calib_line.erase(0, parsed_pos + delimiter.length());
    }
    // z point
    right_xtreme.z = stod(calib_line);

    std::cout << "Now reading the whole control space..." << std::endl;
    std::getline(data_file, calib_line);

    int loop_count = 0;
    delimiter = " ";
    while(std::getline(data_file, calib_line))
    {
        parsed_pos = calib_line.find(delimiter);
        if (parsed_pos != std::string::npos)
        {
            calib_line.erase(0, parsed_pos + delimiter.length());
        }
        parsed_pos = calib_line.find(delimiter);
        if (parsed_pos != std::string::npos)
        {
            parsed_data = calib_line.substr(0, parsed_pos);
            x_profile[loop_count] = stod(parsed_data);
            calib_line.erase(0, parsed_pos + delimiter.length());
        }
        y_profile[loop_count] = stod(calib_line);
        loop_count++;
    }

    std::cout << "Finish loading the calibration data and start to publish calibrated joystick messages." << std::endl;

    // std::cout << "Central point: x = " << central_point.x << ", y = " << central_point.y << ", theta = " << central_point.z << std::endl;
    // std::cout << "Frontmost point: x = " << front_xtreme.x << ", y = " << front_xtreme.y << ", theta = " << front_xtreme.z << std::endl;
    // std::cout << "Backmost point: x = " << back_xtreme.x << ", y = " << back_xtreme.y << ", theta = " << back_xtreme.z << std::endl;
    // std::cout << "Leftmost point: x = " << left_xtreme.x << ", y = " << left_xtreme.y << ", theta = " << left_xtreme.z << std::endl;
    // std::cout << "Backmost point: x = " << right_xtreme.x << ", y = " << right_xtreme.y << ", theta = " << right_xtreme.z << std::endl;

    // for (int i = 0; i < x_profile.size(); ++i)
    // {
    //     // if (x_profile[i] != 0)
    //     // {
    //     //     std::cout << "Angle = " << static_cast<float>(i) / 2 << ", x = " << x_profile[i] << ", y = " << y_profile[i] << std::endl;
    //     // }
    //     std::cout << "Angle = " << static_cast<float>(i) * 5 << ", x = " << x_profile[i] << ", y = " << y_profile[i] << std::endl;
    // }

    raw_subscriber = node_handle.subscribe(raw_data_topic, 1, &rawCallback);
    cmd_publisher = node_handle.advertise<geometry_msgs::Point>(published_topic, 1, false);

    ros::Duration(0.5).sleep();

    cmd_timer = node_handle.createTimer(ros::Duration(0.005), &cmd_timerCallback);

    ros::spin();

    return 0;
}
