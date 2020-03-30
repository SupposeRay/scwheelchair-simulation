/*!
* \mainpage My Personal Index Page
*
* \author Martijn Krijnen
* \version 1.0
* \section intro Introduction
* This node takes in the command velocity
* \section install Installation
* \subsection step1 Step 1: catkin build
*/

#include "vel_safety_filter/vel_safety_filter_node.h"

int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "vel_filter_node");
  ros::NodeHandle node_handle("~");
   
  vel_safety_filter::VelFilterNode VelFilterNode(node_handle); 

  ros::spin();
    
  return 0;
}
