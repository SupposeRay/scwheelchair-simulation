#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <fstream>
#include <thread>
#include <csignal>
#include <atomic>
#include <cmath>
#include <chrono>
#include <exception>

geometry_msgs::Twist cmd_vel;
nav_msgs::Odometry odom;
geometry_msgs::Point joystick_data;
std::atomic<bool> new_mode(false);
bool save = false;
bool save_ref = false;
bool use_sim_time = false;
int e_stop_status;

//ROS params
std::string joystick_topic = "/arduino/joystick";
std::string cmd_vel_topic = "/cmd_vel";
std::string odom_topic = "/odometry/filtered";
std::string base_frame = "base_link";
std::string map_frame = "map";
double frequency = 100;

std::string prev_mode;
std::string mode, main_directory, file_name, reference_file_name;

std::vector<geometry_msgs::Pose> reference_path;
std::vector<geometry_msgs::Pose> user_path;
std::vector<geometry_msgs::Twist> user_input;
std::vector<geometry_msgs::Twist> robot_velocity;
std::vector<geometry_msgs::Point> joystick_input;
std::vector<int> e_stop_status_vec;
std::vector<double> simulation_time;
std::vector<double> system_time;
std::chrono::time_point<std::chrono::system_clock> user_start_sys;
double user_start_ros;

void printModeSelection()
{
    std::cout << "\n\nSelect mode:\n 'r' to record reference path\n 'u' to record user path\n 'd' to stop current recording\n 's' to stop this node\n";
}

void pollUserInput()
{
    while (ros::ok())
    {
        std::string temp_mode;
        if (mode.empty())
            printModeSelection();
        std::cin >> temp_mode;
        std::cout << "Mode selected: " << temp_mode << "\n\n\n";

        if (temp_mode == "s")
        {
            mode = temp_mode;
            return;
        }

        else if (temp_mode == "d" && prev_mode == "r")
        {
            mode = temp_mode;
            std::cout << "Input file name to save the reference path, eg 'reference.txt': \n";
            std::cin >> reference_file_name;
            std::cout << "Saving reference path to " << main_directory << reference_file_name << "\n\n\n";
        }

        else if (temp_mode == "d" && prev_mode == "u")
        {
            mode = temp_mode;
            std::cout << "Input file name to save the full log file, eg 'log.txt': \n";
            std::cin >> file_name;
            std::cout << "Saving all data to " << main_directory << file_name << "\n\n\n";
        }

        else if ((temp_mode == "u" && prev_mode == "r") || (temp_mode == "r" && prev_mode == "u"))
        {
            std::cout << "Tried to change to another recording mode while currently recording, please stop recording ('d') first before changing modes\n";
            printModeSelection();
            continue;
        }

        mode = temp_mode;
        new_mode = true;
    }
}

void signalHandler(int signum)
{
    std::cout << "\nSIGINT caught, closing node.\n";
    if (signum == SIGINT)
        mode = "s";
    exit(signum);
}

void cmdVelCB(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel = *msg;
}

void odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
}

void joystickCB(const geometry_msgs::Point::ConstPtr &msg)
{
    joystick_data = *msg;
}

void estopCB(const std_msgs::Bool::ConstPtr &msg)
{
    e_stop_status = (int)msg->data;
}

void joystickvelCB(const geometry_msgs::Twist &msg)
{
    joystick_data.x = msg.linear.x;
    joystick_data.y = msg.angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scat_logger");
    ros::NodeHandle nh("~");

    //Load parameters
    nh.getParam("frequency", frequency);
    nh.getParam("cmd_vel_topic", cmd_vel_topic);
    nh.getParam("base_frame", base_frame);
    nh.getParam("map_frame", map_frame);
    nh.getParam("odom_topic", odom_topic);
    nh.getParam("/use_sim_time", use_sim_time);
    nh.getParam("joystick_topic", joystick_topic);
    if(use_sim_time)
        ROS_INFO("Use simulation time is enabled");

    ros::Subscriber cmd_vel_sub = nh.subscribe(cmd_vel_topic, 1, cmdVelCB);
    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1, odomCB);
    ros::Subscriber e_stop_sub = nh.subscribe("/e_stop", 1, &estopCB);
    ros::Subscriber joystick_sub;
    if (joystick_topic == "/joy_vel")
    {
        joystick_sub = nh.subscribe(joystick_topic, 1, joystickvelCB);
    }
    else
    {
        joystick_sub = nh.subscribe(joystick_topic, 1, joystickCB);
    }

    tf2_ros::Buffer buf;
    tf2_ros::TransformListener listener(buf);

    //Declare below ros::init as they assign a signal handler as well
    signal(SIGINT, signalHandler);

    //Get main directory to save all log files
    std::string correct;
    while (ros::ok())
    {
        std::cout << "\n\nInput full path to directory (end with '/') for saving log .txt files: \n";
        // main_directory = "/home/ray/path_record/";
        std::cin >> main_directory;
        std::cout << "All log files will be saved to '" << main_directory << "'. Is this correct? (y/n): ";
        std::cin >> correct;
        if (correct == "y")
            break;
        else if (correct != "n")
            std::cout << "Please input 'y' or 'n' only\n\n";
    }
    std::cout << "\n\n";

    //Get previously saved reference path, if any
    std::string ref_path_full_dir;
    while (ros::ok())
    {
        std::cout << "\n\nInput full path to previously saved reference path text file (eg /home/user/ref.txt). Type 'x' and enter if no reference file: \n";
        std::cin >> ref_path_full_dir;

        //No ref file specified
        if (ref_path_full_dir == "x")
            break;

        else
        {
            ref_path_full_dir = main_directory + ref_path_full_dir;
            try
            {
                std::ifstream file;
                file.open(ref_path_full_dir);
                if(!file.is_open())
                    std::cout << "Failed to open reference path text file\n";

                //Read away the header
                std::string in_line;
                std::getline(file, in_line);
                do
                {
                    std::getline(file, in_line);
                    geometry_msgs::Pose new_pose;

                    //Parse comma deliminated string
                    std::string delim = ",";
                    size_t pos = 0;
                    int count = 0;
                    while((pos = in_line.find(delim)) != std::string::npos)
                    {
                        if(count == 0)
                            new_pose.position.x = std::stof(in_line.substr(0, pos));
                        else if(count == 1)
                            new_pose.position.y = std::stof(in_line.substr(0, pos));
                        else if(count == 2)
                            new_pose.position.z = std::stof(in_line.substr(0, pos));
                        else if(count == 3)
                            new_pose.orientation.x = std::stof(in_line.substr(0, pos));
                        else if(count == 4)
                            new_pose.orientation.y = std::stof(in_line.substr(0, pos));
                        else if(count == 5)
                            new_pose.orientation.z = std::stof(in_line.substr(0, pos));
                        else if(count == 6)
                            new_pose.orientation.w = std::stof(in_line.substr(0, pos));

                        in_line.erase(0, pos + delim.length());
                        ++count;
                    }
                    
                    if(!std::isnan(new_pose.position.x) && !std::isnan(new_pose.position.y) && !std::isnan(new_pose.position.z))
                        reference_path.emplace_back(std::move(new_pose));
                } while (!in_line.empty());
            }
            catch (const std::exception &e)
            {
                std::cout << "Load reference path text file failed: \n";
                std::cout << e.what() << "\n";
            }
        }

        if (!reference_path.empty())
        {
            reference_path.pop_back();
            std::cout << "Reference path loaded, " << reference_path.size() << " poses\n";
            break;
        }
    }

    //Start thread to block for user's mode changes
    std::thread user_polling_thread(pollUserInput);

    //Start recording data
    ros::Rate r(frequency);
    while (ros::ok() && mode != "s")
    {
        //If there is a change in mode, cleanup previous mode and then start new mode
        if (new_mode == true)
        {
            if (mode == "d" && prev_mode == "r")
            {
                save_ref = true;
                ROS_INFO("Stopping reference path recording session.");
            }

            else if (mode == "d" && prev_mode == "u")
            {
                save = true;
                ROS_INFO("Stopping user input and path recording session, saving to output file...");
            }

            else if (mode == "r")
            {
                reference_path.clear();
                ROS_INFO("Recording reference path...");
            }

            else if (mode == "u")
            {
                user_path.clear();
                user_input.clear();
                simulation_time.clear();
                system_time.clear();
                robot_velocity.clear();
                joystick_input.clear();
                user_start_sys = std::chrono::system_clock::now();
                if(use_sim_time)
                    user_start_ros = ros::Time::now().toSec();
                ROS_INFO("Recording user path and input...");
            }

            if (mode != "d")
                printModeSelection();

            prev_mode = mode;
            new_mode = false;
        }

        if (!mode.empty())
        {
            if (mode == "r" || mode == "u")
            {
                //Get odometry of robot by getting transform from base_frame to map_frame
                geometry_msgs::TransformStamped odom_to_map_tf;

                try
                {
                    odom_to_map_tf = buf.lookupTransform(map_frame, base_frame, ros::Time(0), ros::Duration(1 / frequency));

                    geometry_msgs::PoseStamped temp_pose;
                    temp_pose.header.frame_id = base_frame;
                    temp_pose.pose.orientation.w = 1;
                    tf2::doTransform(temp_pose, temp_pose, odom_to_map_tf);

                    //If mode is record reference path
                    if (mode == "r")
                    {
                        reference_path.emplace_back(std::move(temp_pose.pose));
                    }

                    //Mode is record user path
                    else if (mode == "u")
                    {
                        ros::spinOnce();
                        user_input.push_back(cmd_vel);
                        user_path.emplace_back(std::move(temp_pose.pose));
                        if(use_sim_time)
                            simulation_time.emplace_back(ros::Time::now().toSec() - user_start_ros);
                        else
                            simulation_time.emplace_back((std::chrono::system_clock::now() - user_start_sys).count() / 1000000000.0);
                        system_time.emplace_back((std::chrono::system_clock::now() - user_start_sys).count() / 1000000000.0);
                        robot_velocity.push_back(odom.twist.twist);
                        joystick_input.push_back(joystick_data);
                        e_stop_status_vec.push_back(e_stop_status);
                    }
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_ERROR("%s", ex.what());
                }
            }

            //Save all data into log file then clear
            if (save)
            {
                ROS_INFO("Reference path size %ld, user path size %ld", reference_path.size(), user_path.size());
                std::ofstream file;
                file.open(main_directory + file_name);
                file << "ref.x,ref.y,ref.z,ref.o.x,ref.o.y,ref.o.z,ref.o.w,sim.time,user.x,user.y,user.z,user.o.x,user.o.y,user.o.z,user.o.w,user.linvel,user.angvel,robot.linvel,robot.angvel,joy.x,joy.y,sys.time,estopstatus\n";
                for (int i = 0; i < reference_path.size() || i < user_path.size(); ++i)
                {
                    char buff[300];
                    sprintf(buff, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
                            i < reference_path.size() ? reference_path[i].position.x : std::nan(""),
                            i < reference_path.size() ? reference_path[i].position.y : std::nan(""),
                            i < reference_path.size() ? reference_path[i].position.z : std::nan(""),
                            i < reference_path.size() ? reference_path[i].orientation.x : std::nan(""),
                            i < reference_path.size() ? reference_path[i].orientation.y : std::nan(""),
                            i < reference_path.size() ? reference_path[i].orientation.z : std::nan(""),
                            i < reference_path.size() ? reference_path[i].orientation.w : std::nan(""),
                            i < simulation_time.size() ? simulation_time[i] : std::nan(""),
                            i < user_path.size() ? user_path[i].position.x : std::nan(""),
                            i < user_path.size() ? user_path[i].position.y : std::nan(""),
                            i < user_path.size() ? user_path[i].position.z : std::nan(""),
                            i < user_path.size() ? user_path[i].orientation.x : std::nan(""),
                            i < user_path.size() ? user_path[i].orientation.y : std::nan(""),
                            i < user_path.size() ? user_path[i].orientation.z : std::nan(""),
                            i < user_path.size() ? user_path[i].orientation.w : std::nan(""),
                            i < user_input.size() ? user_input[i].linear.x : std::nan(""),
                            i < user_input.size() ? user_input[i].angular.z : std::nan(""),
                            i < robot_velocity.size() ? robot_velocity[i].linear.x : std::nan(""),
                            i < robot_velocity.size() ? robot_velocity[i].angular.z : std::nan(""),
                            i < joystick_input.size() ? joystick_input[i].x : std::nan(""),
                            i < joystick_input.size() ? joystick_input[i].y : std::nan(""),
                            i < system_time.size() ? system_time[i] : std::nan(""),
                            i < e_stop_status_vec.size() ? e_stop_status_vec[i] : -1);
                    std::string output(buff);
                    file << output;
                }

                ROS_INFO("Save complete, you can now restart any other modes");
                ROS_INFO("Running 'u' from here will not clear the previously recorded reference path");
                printModeSelection();
                save = false;
                file.close();
            }

            if(save_ref)
            {
                std::ofstream file;
                file.open(main_directory + reference_file_name);
                file << "ref.x,ref.y,ref.z,ref.o.x,ref.o.y,ref.o.z,ref.o.w\n";
                for (int i = 0; i < reference_path.size(); ++i)
                {
                    char buff[150];
                    sprintf(buff, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                            i < reference_path.size() ? reference_path[i].position.x : std::nan(""),
                            i < reference_path.size() ? reference_path[i].position.y : std::nan(""),
                            i < reference_path.size() ? reference_path[i].position.z : std::nan(""),
                            i < reference_path.size() ? reference_path[i].orientation.x : std::nan(""),
                            i < reference_path.size() ? reference_path[i].orientation.y : std::nan(""),
                            i < reference_path.size() ? reference_path[i].orientation.z : std::nan(""),
                            i < reference_path.size() ? reference_path[i].orientation.w : std::nan(""));
                    std::string output(buff);
                    file << output;
                }
                ROS_INFO("Reference path save complete, you can now restart any other modes");
                ROS_INFO("Running 'u' from here will not clear the recorded & saved reference path");
                printModeSelection();
                save_ref = false;
                file.close();
            }
        }

        r.sleep();
    }

    mode = "s";
    user_polling_thread.join();
    return 1;
}
