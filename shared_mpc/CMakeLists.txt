cmake_minimum_required(VERSION 2.8.3)
project(shared_mpc)

## Compiler
ADD_DEFINITIONS( -std=c++11 )

## IPOPT
#FIND_PACKAGE( IPOPT REQUIRED )
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  scat_libs
  scat_msgs
  sparse_voronoi
  obstacle_detector
  laser_line_extraction
  laserline
)

catkin_package(
  INCLUDE_DIRS 
	include
#  LIBRARIES 
#	${PROJECT_NAME}_lib
  CATKIN_DEPENDS 
	message_runtime 
	roscpp 
	scat_libs
	scat_msgs
	sparse_voronoi 
	obstacle_detector
	laser_line_extraction 
	laserline 
)

###########
## Build ##
###########
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Debug)

#FILE(GLOB_RECURSE CURRENT_INCLUDE include/*.h)
# FILE(GLOB_RECURSE CURRENT_SOURCES src/*.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${sparse_voronoi_INCLUDE_DIRS}
  ${scat_libs_INCLUDE_DIRS}
#  ${obstacle_detector_INCLUDE_DIRS}
  ${laser_line_extraction_INCLUDE_DIRS}
  ${laserline_INCLUDE_DIRS}
)

## Declare cpp libaries
#add_library(${PROJECT_NAME}_lib src/mpc_node_velocity.cpp src/fg_eval_velocity.cpp)
## Add dependencies to exported targets, like ROS msgs or srvs
#add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ipopt )

## Create executable 
add_executable(  ${PROJECT_NAME}
src/mpc_main_velocity.cpp 
src/mpc_node_velocity.cpp 
src/fg_eval_diff_drive.cpp
${${PROJECT_NAME}_SOURCES})

## Specify libraries to link executable targets against
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ipopt) #${PROJECT_NAME}_lib
