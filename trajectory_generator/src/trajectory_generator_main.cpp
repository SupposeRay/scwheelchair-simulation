#include "trajectory_generator/trajectory_generator_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle node_handle("~");

  trajectory_generator::TrajectoryGeneratorNode TrajectoryGeneratorNode(node_handle);

  ros::spin();

  return 0;
}