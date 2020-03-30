#include "scans_merger/scans_merger.h"

using namespace scans_merger;

int main(int argc, char** argv) {
  ros::init(argc, argv, "scans_merger", ros::init_options::NoRosout);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try {
    ROS_INFO("[Scans Merger]: Initializing node");
    ScansMerger sm(nh, nh_local);
    ros::spin();
  }
  catch (const char* s) {
    ROS_FATAL_STREAM("[Scans Merger]: " << s);
  }
  catch (...) {
    ROS_FATAL_STREAM("[Scans Merger]: Unexpected error");
  }

  return 0;
}
