cmake_minimum_required(VERSION 2.8.3)
project(test_cmd_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  message_generation
  actionlib
  move_base_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy geometry_msgs message_runtime
#  DEPENDS
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

#############
## Scripts ##
#############

add_executable(test_cmd_publisher src/test_cmd_publisher.cpp)
target_link_libraries(test_cmd_publisher ${catkin_LIBRARIES})

