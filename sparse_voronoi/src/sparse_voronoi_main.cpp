#include "sparse_voronoi/sparse_voronoi_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sparse_voronoi_node");
  ros::NodeHandle node_handle("~");
   
  sparse_voronoi::VoroNode VoroNode(node_handle); 

  ros::spin();
    
  return 0;
}
