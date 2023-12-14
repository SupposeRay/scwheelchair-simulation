// ROS
#include <ros/ros.h>
// Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
// Visualization
// #include <visualization_msgs/Marker.h>
// C++
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>

ros::Publisher joystick_publisher;
ros::Publisher velocity_publisher;
ros::Timer timer;

std::vector<geometry_msgs::Point> joystick_data;
std::vector<geometry_msgs::Twist> velocity_data;
geometry_msgs::Point empty_cmd;
geometry_msgs::Twist empty_vel;

float pub_freq = 100;
bool data_receive = false;
int data_idx = 0;

void TimerCallback(const ros::TimerEvent &)
{
  if (data_receive)
  {
    int data_size = std::min(joystick_data.size(), velocity_data.size());
    if (data_idx < data_size)
    {
      joystick_publisher.publish(joystick_data[data_idx]);
      velocity_publisher.publish(velocity_data[data_idx]);
      data_idx++;
    }
    else
    {
      joystick_publisher.publish(empty_cmd);
      velocity_publisher.publish(empty_vel);
      data_receive = false;
      data_idx = 0;
      std::cout << "Finish publishing the data." << std::endl;
      ROS_WARN_STREAM("Shutting down ROS...");
      ros::requestShutdown();
    }
  }
}

int main(int argc, char **argv)
{
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  ros::init(argc, argv, "data_reproduction");
  ros::NodeHandle node_handle("~");

  /* Read the parameter */
  if (!node_handle.getParam("publish_frequency", pub_freq))
  {
    ROS_WARN_STREAM("Parameter publish_frequency not set. Using default setting: " << pub_freq);
  }

  /* Subscribers, publishers and timer */
  joystick_publisher = node_handle.advertise<geometry_msgs::Point>("/arduino/joystick", 1);
  velocity_publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  timer = node_handle.createTimer(ros::Duration(1 / pub_freq), TimerCallback);

  /* Load the file */
  std::string file_directory;

  std::cout << "Please enter the full path directory (without \"~\") to the log file including extension:" << std::endl;

  std::getline(std::cin, file_directory);

  std::cout<< "Loading " << file_directory << " ..." << std::endl;

  std::ifstream log_file;
  log_file.open(file_directory, std::ifstream::in);

  if (!log_file.is_open())
	{
    std::cout << "Unable to open file, please check if the file exists or it is permitted to read." << std::endl;
		ROS_ERROR_STREAM("Shutting down ROS...");
    ros::requestShutdown();
	}
  else
  {
    std::cout << "File successfully loaded." << std::endl;
  }
  
  /* Read the file */

  geometry_msgs::Point joystick_input;
  geometry_msgs::Twist velocity_input;
  std::string file_line, data_string;
  std::vector<float> data_value;

  std::getline(log_file, file_line);  // read the first line to remove the header

  while(std::getline(log_file, file_line))
  {
    std::stringstream stream_line(file_line);
    
    while(std::getline(stream_line, data_string, ','))
    {
      data_value.push_back(std::stof(data_string));
    }

    if (std::isnan(data_value[19]) || std::isnan(data_value[19]))
    {
      break;
    }

    velocity_input.linear.x = data_value[15];
    velocity_input.angular.z = data_value[16];
    velocity_data.push_back(velocity_input);

    joystick_input.x = data_value[19];
    joystick_input.y = data_value[20];
    joystick_data.push_back(joystick_input);
    data_value.clear();
  }
  
  /* Finish file reading */
  if (!joystick_data.empty() && !velocity_data.empty())
  {
    data_receive = true;
  }

  ros::spin();
  
  return 0;
}