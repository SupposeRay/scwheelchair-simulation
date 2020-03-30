cmake_minimum_required(VERSION 2.8.3)
project(scat_libs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf
  scat_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}_lib
  CATKIN_DEPENDS 
	roscpp geometry_msgs std_msgs tf scat_msgs sensor_msgs nav_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_lib
  src/base_utils.cpp
  src/geom_utils.cpp    
  src/tf_utils.cpp  
  src/nav_utils.cpp
  src/obst_dist.cpp
  src/rosmsg.cpp
  src/diff_drive.cpp  
  src/obst_utils.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)
