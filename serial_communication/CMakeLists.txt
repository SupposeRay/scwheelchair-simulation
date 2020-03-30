cmake_minimum_required(VERSION 2.8.3)
project(serial_communication)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  scat_libs
)


catkin_package(
CATKIN_DEPENDS
	scat_libs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_communication src/serial_communication.cpp)
target_link_libraries(serial_communication ${catkin_LIBRARIES})


