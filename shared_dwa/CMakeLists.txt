cmake_minimum_required(VERSION 2.8.3)
project(shared_dwa)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
  tf2
  tf2_ros
  obstacle_detector
  path_belief_update
)

find_package(OpenCV)

# add_service_files(
#    FILES
#    doorway_srv.srv
# )

add_action_files(
  DIRECTORY action
  FILES
  doorway_detection.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge actionlib actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

FILE(GLOB_RECURSE CURRENT_INCLUDE include/*.h)
FILE(GLOB_RECURSE CURRENT_SOURCES src/*.cpp)

add_executable(${PROJECT_NAME}
  ${CURRENT_SOURCES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  path_belief_update_generate_messages_cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)