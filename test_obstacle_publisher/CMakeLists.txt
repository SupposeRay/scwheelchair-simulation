cmake_minimum_required(VERSION 2.8.3)
project(test_obstacle_publisher)

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
  visualization_msgs
  scat_msgs
  scat_libs
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS message_runtime roscpp std_msgs visualization_msgs scat_msgs scat_libs
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

add_executable(${PROJECT_NAME}
  src/test_obstacle_publisher_main.cpp
  src/test_obstacle_publisher_node.cpp
)


## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

