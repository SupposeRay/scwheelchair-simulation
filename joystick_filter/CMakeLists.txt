cmake_minimum_required(VERSION 2.8.3)
project(joystick_filter)

## Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  scat_libs
)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}_lib
  CATKIN_DEPENDS message_runtime roscpp std_msgs scat_libs
  DEPENDS EIGEN3
)

###########
## Build ##
###########
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Debug)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_lib
   src/${PROJECT_NAME}_node.cpp
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_main.cpp
  src/${PROJECT_NAME}_node.cpp  
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES}) # Eigen3::Eigen
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib ${catkin_LIBRARIES} ) # Eigen3::Eigen

## TESTING
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.test src/test/test_${PROJECT_NAME}.cpp)
#  target_link_libraries(test_${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
#endif()





