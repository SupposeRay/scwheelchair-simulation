#include "input_converter/input_converter_node.h"

int main(int argc, char **argv)
{
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  ros::init(argc, argv, "input_converter");
  ros::NodeHandle node_handle("~");  

  input_converter::input_converterNode Input_Converter(node_handle);

  ros::spin();

  return 0;
}