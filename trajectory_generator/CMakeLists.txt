cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  cv_bridge
)

find_package(OpenCV)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp cv_bridge
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
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)