#include <scat_master/keyboard_node.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_node");
  ros::NodeHandle node_handle("~");
  keyboard_node::KeyboardNode KeyboardNode(node_handle);
  signal(SIGINT,keyboard_node::shutdown);  
  ros::spin();

  return 0;
}

namespace keyboard_node
{

// Global Variables required for keyboard. 
int kfd = 0;
struct termios cooked, raw;

KeyboardNode::KeyboardNode(ros::NodeHandle &nodeHandle)
    : node_handle_(nodeHandle)
{

  timer_ = node_handle_.createTimer(ros::Duration(0.2), &KeyboardNode::keyboardCallback, this);
  mode_publisher_ = node_handle_.advertise<std_msgs::String>("/scat_master/set_mode",1);  

  // Prepare keyboard:
  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  ROS_INFO("Press 'Shift + A' to switch to Autonomous mode");
  ROS_INFO("Press 'Shift + S' to switch to Shared mode");
  ROS_INFO("Press 'Shift + D' to switch to DIY (manual) mode");
}

KeyboardNode::~KeyboardNode() {}

// callback for keyboard, can be used to switch between modes. 
void KeyboardNode::keyboardCallback(const ros::TimerEvent &)
{
  std_msgs::String mode_msg;

  char c;
  // get the next event from the keyboard  
  if(read(kfd, &c, 1) < 0)
  {
    perror("read():");
    exit(-1);
  }

  ROS_DEBUG("value: 0x%02X\n", c);

  switch(c)
  {
    case KEYCODE_A:
      ROS_INFO("Keyboard command 'Autonomous'. Switching to Autonomous mode.");
      mode_msg.data = "autonomous";
      mode_publisher_.publish(mode_msg);
      break;
    case KEYCODE_S:
      ROS_INFO("Keyboard command 'Shared'. Switching to Shared Control mode.");
      mode_msg.data = "shared";
      mode_publisher_.publish(mode_msg);
      break;
    case KEYCODE_D:
      ROS_INFO("Keyboard command 'Do-It-Yourself'. Switching to Manual Control mode.");
      mode_msg.data = "manual";
      mode_publisher_.publish(mode_msg);
      break;
  }
}

void shutdown(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

} // namespace keyboard_node
