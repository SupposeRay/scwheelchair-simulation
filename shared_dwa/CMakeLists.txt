cmake_minimum_required(VERSION 2.8.3)
project(shared_dwa)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  roslib
  tf
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  scat_msgs
  scat_libs
)

find_package(OpenCV)

catkin_package(
  CATKIN_DEPENDS roscpp costmap_2d roslib tf geometry_msgs nav_msgs std_msgs scat_msgs scat_libs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

FILE(GLOB_RECURSE CURRENT_INCLUDE include/shared_dwa/*.h)
FILE(GLOB_RECURSE CURRENT_SOURCES src/*.cpp)

add_executable(${PROJECT_NAME}
  ${CURRENT_SOURCES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

