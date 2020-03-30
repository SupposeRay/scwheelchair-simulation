#include "shared_mpc/mpc_node_velocity.h"

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "shared_mpc");
  ros::NodeHandle node_handle("~");
   
  shared_mpc::MPCNode MPCNode(node_handle); 

  ros::spin();
    
  return 0;
}
