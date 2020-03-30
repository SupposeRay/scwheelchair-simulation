cmake_minimum_required(VERSION 2.8.3)
project(vel_safety_filter)

## Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  laser_geometry
  visualization_msgs
  tf
  scat_libs
  scat_msgs
  pcl_ros
) 

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}_lib
  CATKIN_DEPENDS   
	roscpp
	std_msgs
	sensor_msgs
	geometry_msgs
	nav_msgs
	laser_geometry
	visualization_msgs
	tf
	scat_libs
	scat_msgs
        pcl_ros
)

###########
## Build ##
###########
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Debug)
FILE(GLOB_RECURSE CURRENT_INCLUDE include/*.h)
FILE(GLOB_RECURSE CURRENT_SOURCES src/*.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_lib
   src/${PROJECT_NAME}_node.cpp
)

add_executable(${PROJECT_NAME}
   src/${PROJECT_NAME}_main.cpp
   src/${PROJECT_NAME}_node.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
