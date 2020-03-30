#ifndef KEYBOARD_NODE_H
#define KEYBOARD_NODE_H

#include <ros/ros.h>
// Standard messages
#include <std_msgs/String.h>
// C++
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
 
// Global declarations required for keyboard handling
#define KEYCODE_A 0x41
#define KEYCODE_S 0x53
#define KEYCODE_D 0x44

namespace keyboard_node
{

// shutdown function, required to restore keyboard input and shutdown ros before stopping node process. 
void shutdown(int sig);

class KeyboardNode
{
public:
  // Constructor
  KeyboardNode(ros::NodeHandle &nodeHandle);

  // Destructor
  virtual ~KeyboardNode();

private:
  // callback for keyboard, can be used to switch between modes. 
  void keyboardCallback(const ros::TimerEvent &);

  // ROS objects
  ros::NodeHandle &node_handle_;
  ros::Timer timer_;
  ros::Publisher mode_publisher_;
};

} // namespace keyboard_node
#endif
