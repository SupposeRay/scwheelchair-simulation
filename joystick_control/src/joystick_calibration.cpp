#include <ros/ros.h>
#include <chrono>
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
ros::AsyncSpinner *spinner_;

std::ofstream data_file;
std::string raw_data_topic = "/raw_data";
std::string calib_path = "/home/mrca/";
std::string calib_name = "joystick_calibration.txt";
std::string keyborad_input = "n";

geometry_msgs::Point raw_msg;   // use x & y to store the joystick's raw x & y values, use z to store the raw angle value
bool raw_received = false;

geometry_msgs::Point front_xtreme;
geometry_msgs::Point back_xtreme;
geometry_msgs::Point left_xtreme;
geometry_msgs::Point right_xtreme;
geometry_msgs::Point central_point;

std::vector<double> x_profile;
std::vector<double> y_profile;
std::vector<int> profile_count;

std::chrono::time_point<std::chrono::system_clock> start_time;
std::chrono::time_point<std::chrono::system_clock> current_time;
double time_duration = 0;

void rawCallback(const geometry_msgs::Point::ConstPtr &msg_raw)
{
    raw_msg = *msg_raw;
    raw_received = true;
}

int main(int argc, char** argv)
{
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::init(argc, argv, "joystick_calibration");
    auto node_name = ros::this_node::getName();
    std::cout << node_name << " started." << std::endl;
    ros::NodeHandle node_handle("~");
    node_handle.getParam("raw_data_topic", raw_data_topic);
    node_handle.getParam("calib_path", calib_path);
    node_handle.getParam("calib_name", calib_name);
    spinner_ = new ros::AsyncSpinner(0);

    x_profile.resize(72, 0.0);
    y_profile.resize(72, 0.0);
    profile_count.resize(72, 0);

    raw_subscriber = node_handle.subscribe(raw_data_topic, 1, &rawCallback);

    ros::Duration(0.5).sleep();

    std::cout << "Starting to calibrate the joystick..." << std::endl;
    spinner_->start();
    int loop_count = 0;
    data_file.open(calib_path + calib_name);

    if (!raw_received)
    {
        ROS_ERROR_STREAM("ERROR! Joystick raw data not received!");
        std::cout << "Shutting down the node..." << std::endl;
        data_file.close();
        spinner_->stop();
        ros::shutdown();
        exit(0);
    }

    /* The central point */

    std::cout << "First, DO NOT TOUCH the joystick for 2 secs to get the central point." << std::endl;

    while (!(keyborad_input == "y" || keyborad_input == "Y"))
    {
        std::cout << "Is the joystick released? y/n" << std::endl;
        std::cin >> keyborad_input;
    }
    std::cout << "Starting to calibrate the central point." << std::endl;
    start_time = std::chrono::system_clock::now();
    current_time = std::chrono::system_clock::now();
    time_duration = (current_time - start_time).count() / 1000000000.0;
    do
    {
        central_point.x += raw_msg.x;
        central_point.y += raw_msg.y;
        central_point.z += raw_msg.z;

        loop_count++;
        current_time = std::chrono::system_clock::now();
        time_duration = (current_time - start_time).count() / 1000000000.0;
    }
    while (time_duration <= 2.0);

    if (loop_count == 0)
    {
        ROS_ERROR_STREAM("ERROR! Fail to get the central point, please check the code!");
        std::cout << "Shutting down the node..." << std::endl;
        data_file.close();
        spinner_->stop();
        ros::shutdown();
        exit(0);
    }
    else
    {
        central_point.x /= static_cast<double>(loop_count);
        central_point.y /= static_cast<double>(loop_count);
        central_point.z /= static_cast<double>(loop_count);

        data_file << "Central Point: ";
        data_file << central_point.x << ", ";
        data_file << central_point.y << ", ";
        data_file << central_point.z << std::endl;
    }

    std::cout << "Finished the calibration of the central point." << std::endl;
    keyborad_input = "n";
    loop_count = 0;

    /* The frontmost point */

    std::cout << "Next, place the joystick at the extreme front position and wiggle it in left/right directions for 2 secs." << std::endl;
    while (!(keyborad_input == "y" || keyborad_input == "Y"))
    {
        std::cout << "Is the joystick pushed to extreme front position? y/n" << std::endl;
        std::cin >> keyborad_input;
    }
    std::cout << "Starting to calibrate, keep the extreme poisition and wiggle the joystick left and right." << std::endl;
    start_time = std::chrono::system_clock::now();
    current_time = std::chrono::system_clock::now();
    time_duration = (current_time - start_time).count() / 1000000000.0;
    do
    {
        if (raw_msg.z <= 2 || raw_msg.z >= 358)
        {
            front_xtreme.x += raw_msg.x;
            front_xtreme.y += raw_msg.y;
            loop_count++;
        }
        current_time = std::chrono::system_clock::now();
        time_duration = (current_time - start_time).count() / 1000000000.0;
    }
    while (time_duration <= 2.0);

    if (loop_count == 0)
    {
        ROS_ERROR_STREAM("ERROR! Fail to get the frontmost point, please check the code!");
        std::cout << "Shutting down the node..." << std::endl;
        data_file.close();
        spinner_->stop();
        ros::shutdown();
        exit(0);
    }
    else
    {
        front_xtreme.x /= static_cast<double>(loop_count);
        front_xtreme.y /= static_cast<double>(loop_count);
        front_xtreme.z = 0;
        
        data_file << "Frontmost Point: ";
        data_file << front_xtreme.x << ", ";
        data_file << front_xtreme.y << ", ";
        data_file << front_xtreme.z << std::endl;
    }

    std::cout << "Finished the calibration of the frontmost point." << std::endl;
    keyborad_input = "n";
    loop_count = 0;

    std::cout << "Now, release the joystick." << std::endl;

    /* The backmost point */

    std::cout << "Next, place the joystick at the extreme back position and wiggle it in left/right directions for 2 secs." << std::endl;
    while (!(keyborad_input == "y" || keyborad_input == "Y"))
    {
        std::cout << "Is the joystick pulled to extreme back position? y/n" << std::endl;
        std::cin >> keyborad_input;
    }
    std::cout << "Starting to calibrate, keep the extreme poisition and wiggle the joystick left and right." << std::endl;
    start_time = std::chrono::system_clock::now();
    current_time = std::chrono::system_clock::now();
    time_duration = (current_time - start_time).count() / 1000000000.0;
    do
    {
        if (raw_msg.z >= 178 && raw_msg.z <= 182)
        {
            back_xtreme.x += raw_msg.x;
            back_xtreme.y += raw_msg.y;
            loop_count++;
        }
        current_time = std::chrono::system_clock::now();
        time_duration = (current_time - start_time).count() / 1000000000.0;
    }
    while (time_duration <= 2.0);

    if (loop_count == 0)
    {
        ROS_ERROR_STREAM("ERROR! Fail to get the backmost point, please check the code!");
        std::cout << "Shutting down the node..." << std::endl;
        data_file.close();
        spinner_->stop();
        ros::shutdown();
        exit(0);
    }
    else
    {
        back_xtreme.x /= static_cast<double>(loop_count);
        back_xtreme.y /= static_cast<double>(loop_count);
        back_xtreme.z = 180;

        data_file << "Backmost Point: ";
        data_file << back_xtreme.x << ", ";
        data_file << back_xtreme.y << ", ";
        data_file << back_xtreme.z << std::endl;
    }

    std::cout << "Finished the calibration of the backmost point." << std::endl;
    keyborad_input = "n";
    loop_count = 0;

    std::cout << "Now, release the joystick." << std::endl;

    /* The leftmost point */

    std::cout << "Next, place the joystick at the extreme left position and wiggle it in front/back directions for 2 secs." << std::endl;
    while (!(keyborad_input == "y" || keyborad_input == "Y"))
    {
        std::cout << "Is the joystick pushed to extreme left position? y/n" << std::endl;
        std::cin >> keyborad_input;
    }
    std::cout << "Starting to calibrate, keep the extreme poisition and wiggle the joystick front and back." << std::endl;
    start_time = std::chrono::system_clock::now();
    current_time = std::chrono::system_clock::now();
    time_duration = (current_time - start_time).count() / 1000000000.0;
    do
    {
        if (raw_msg.z >= 88 && raw_msg.z <= 92)
        {
            left_xtreme.x += raw_msg.x;
            left_xtreme.y += raw_msg.y;
            loop_count++;
        }
        current_time = std::chrono::system_clock::now();
        time_duration = (current_time - start_time).count() / 1000000000.0;
    }
    while (time_duration <= 2.0);

    if (loop_count == 0)
    {
        ROS_ERROR_STREAM("ERROR! Fail to get the leftmost point, please check the code!");
        std::cout << "Shutting down the node..." << std::endl;
        data_file.close();
        spinner_->stop();
        ros::shutdown();
        exit(0);
    }
    else
    {
        left_xtreme.x /= static_cast<double>(loop_count);
        left_xtreme.y /= static_cast<double>(loop_count);
        left_xtreme.z = 90;

        data_file << "Leftmost Point: ";
        data_file << left_xtreme.x << ", ";
        data_file << left_xtreme.y << ", ";
        data_file << left_xtreme.z << std::endl;
    }

    std::cout << "Finished the calibration of the leftmost point." << std::endl;
    keyborad_input = "n";
    loop_count = 0;

    std::cout << "Now, release the joystick." << std::endl;
    
    /* The rightmost point */

    std::cout << "Next, place the joystick at the extreme right position and wiggle it in front/back directions for 2 secs." << std::endl;
    while (!(keyborad_input == "y" || keyborad_input == "Y"))
    {
        std::cout << "Is the joystick pushed to extreme right position? y/n" << std::endl;
        std::cin >> keyborad_input;
    }
    std::cout << "Starting to calibrate, keep the extreme poisition and wiggle the joystick front and back." << std::endl;
    start_time = std::chrono::system_clock::now();
    current_time = std::chrono::system_clock::now();
    time_duration = (current_time - start_time).count() / 1000000000.0;
    do
    {
        if (raw_msg.z >= 268 && raw_msg.z <= 272)
        {
            right_xtreme.x += raw_msg.x;
            right_xtreme.y += raw_msg.y;
            loop_count++;
        }
        current_time = std::chrono::system_clock::now();
        time_duration = (current_time - start_time).count() / 1000000000.0;
    }
    while (time_duration <= 2.0);

    if (loop_count == 0)
    {
        ROS_ERROR_STREAM("ERROR! Fail to get the rightmost point, please check the code!");
        std::cout << "Shutting down the node..." << std::endl;
        data_file.close();
        spinner_->stop();
        ros::shutdown();
        exit(0);
    }
    else
    {
        right_xtreme.x /= static_cast<double>(loop_count);
        right_xtreme.y /= static_cast<double>(loop_count);
        right_xtreme.z = 270;

        data_file << "Rightmost Point: ";
        data_file << right_xtreme.x << ", ";
        data_file << right_xtreme.y << ", ";
        data_file << right_xtreme.z << std::endl;
    }

    std::cout << "Finished the calibration of the rightmost point." << std::endl;
    keyborad_input = "n";
    loop_count = 0;

    std::cout << "Now, release the joystick." << std::endl;
    
    /* The whole control space */

    std::cout << "Next, rotate the joystick along the outermost ring of the control space for 10 secs." << std::endl;
    while (!(keyborad_input == "y" || keyborad_input == "Y"))
    {
        std::cout << "Is the joystick rotating along the outermost circle now? y/n" << std::endl;
        std::cin >> keyborad_input;
    }
    std::cout << "Starting to calibrate, keep rotating the joystick along the outermost circle at a steady pace." << std::endl;
    data_file << "Whole control space:" << std::endl;
    start_time = std::chrono::system_clock::now();
    current_time = std::chrono::system_clock::now();
    time_duration = (current_time - start_time).count() / 1000000000.0;
    std::vector<int>::iterator it;
    int extension_times = 0;
    do
    {
        loop_count = round(raw_msg.z / 5);
        loop_count = loop_count == 72 ? 0 : loop_count;
        x_profile[loop_count] += raw_msg.x;
        y_profile[loop_count] += raw_msg.y;
        profile_count[loop_count]++;
        it = std::find(profile_count.begin(), profile_count.end(), 0);

        current_time = std::chrono::system_clock::now();
        time_duration = (current_time - start_time).count() / 1000000000.0;

        time_duration -= 5 * extension_times;

        if (time_duration > 10.0 && it != profile_count.end())
        {
            std::cout << "Finished 10 secs but angle " << (it - profile_count.begin()) * 5 << " is not covered!" << std::endl;
            std::cout << "Rotate slowly and steady towards that angle for another 5 secs!" << std::endl;
            extension_times++;
        }
    }
    while (time_duration <= 10.0 || it != profile_count.end());

    std::cout << "Finished the calibration of the whole control space." << std::endl;
    std::cout << "Now, release the joystick." << std::endl;

    for (int i = 0; i < profile_count.size(); ++i)
    {
        x_profile[i] /= static_cast<double>(profile_count[i]);
        y_profile[i] /= static_cast<double>(profile_count[i]);

        data_file << std::setprecision(5) << static_cast<float>(i) * 5 << " ";
        data_file << x_profile[i] << " ";
        data_file << y_profile[i] << std::endl;
    }

    data_file.close();
    std::cout << "Calibration file saved." << std::endl;

    std::cout << "Quitting the node..." << std::endl;
    spinner_->stop();
    ros::shutdown();
    exit(0);

    return 0;
    // keyborad_input = "n";
    // loop_count = 0;


    // cmd_timer = node_handle.createTimer(ros::Duration(cmd_interval), &cmd_timerCallback);
    // record_timer = node_handle.createTimer(ros::Duration(0.01), &record_timerCallback);

    // ros::spin();

    // ros::spin();
}