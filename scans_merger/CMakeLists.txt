cmake_minimum_required(VERSION 2.8.3)
project(scans_merger)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS roscpp roslaunch nodelet rviz std_msgs std_srvs geometry_msgs sensor_msgs tf laser_geometry )
find_package(Boost 1.54.0 REQUIRED system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scans_merger
  CATKIN_DEPENDS roscpp nodelet rviz std_msgs std_srvs geometry_msgs sensor_msgs tf laser_geometry message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS}${Boost_INCLUDE_DIRS})

#
# Build libs
#
add_library(scans_merger src/scans_merger.cpp)
target_link_libraries(scans_merger ${catkin_LIBRARIES})
add_dependencies(scans_merger ${catkin_EXPORTED_TARGETS})

#
# Build nodes
#
add_executable(scans_merger_node src/scans_merger_node.cpp)
target_link_libraries(scans_merger_node scans_merger)


#
# Install libraries
#
#install(TARGETS scans_merger 
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#
# Install nodes
#
#install(TARGETS scans_merger_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#
# Install header files
#
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#
# Install launch files
#
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

