cmake_minimum_required(VERSION 3.0.2)
project(joystick_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wheelchair_vel_test
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(joystick_calibration src/joystick_calibration.cpp)
add_dependencies(joystick_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_calibration ${catkin_LIBRARIES})

add_executable(joystick_control src/joystick_control.cpp)
add_dependencies(joystick_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_control ${catkin_LIBRARIES})