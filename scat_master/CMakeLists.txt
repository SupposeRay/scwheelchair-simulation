cmake_minimum_required(VERSION 2.8.3)
project(scat_master)

## Use C++11
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  scat_msgs
  message_generation
)

catkin_package(
 INCLUDE_DIRS 
	include
 LIBRARIES 
 CATKIN_DEPENDS 
  	roscpp std_msgs std_srvs message_runtime scat_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(scat_master
  src/master_node.cpp
)
add_executable(keyboard_node
  src/keyboard_node.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(scat_master ${catkin_EXPORTED_TARGETS})
add_dependencies(keyboard_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link executable targets against
target_link_libraries(scat_master ${catkin_LIBRARIES} )
target_link_libraries(keyboard_node ${catkin_LIBRARIES} )

