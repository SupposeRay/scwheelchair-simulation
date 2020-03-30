cmake_minimum_required(VERSION 2.8.3)
project(sparse_voronoi)

## Compiler
ADD_DEFINITIONS( -std=c++11 )

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  laserline
  scat_libs
  scat_msgs
  obstacle_detector
  laser_line_extraction
  laserline
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}_lib
  CATKIN_DEPENDS 
	message_runtime 
	roscpp 
	scat_libs
	scat_msgs 
	obstacle_detector
	laserline 
	laser_line_extraction 
)

###########
## Build ##
###########
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Debug)

#FILE(GLOB_RECURSE CURRENT_INCLUDE include/sparse_voronoi/*.h)
#FILE(GLOB_RECURSE CURRENT_SOURCES src/*.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${scat_libs_INCLUDE_DIRS}
#  ${obstacle_detector_INCLUDE_DIRS}
  ${laser_line_extraction_INCLUDE_DIRS}
  ${laserline_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_lib
   src/${PROJECT_NAME}_node.cpp
   src/voronoi_algorithm.cpp
   src/CDT.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_main.cpp
  src/${PROJECT_NAME}_node.cpp  
  src/voronoi_algorithm.cpp
  src/CDT.cpp
)

## Specify libraries to link executable targets against
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)
