cmake_minimum_required(VERSION 2.8.3)
project(laserline)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    rosconsole
    roscpp
    sensor_msgs
    rospy
    std_msgs
    scat_msgs
    #system
    #thread
    #Boost
    #self_test
    #diagnostic_updater
    #driver_base
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS rosconsole
    LIBRARIES ${PROJECT_NAME}_lib
    CATKIN_DEPENDS rosconsole scat_msgs roslib roscpp
)

#find_package(PkgConfig REQUIRED)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Debug)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

FILE(GLOB_RECURSE CURRENT_INCLUDE include/*.h)
FILE(GLOB_RECURSE CURRENT_SOURCES src/*.cpp)

add_executable(${PROJECT_NAME}
  ${CURRENT_SOURCES}
)

add_library(${PROJECT_NAME}_lib
  ${CURRENT_SOURCES}
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_lib 
${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME} 
${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_lib 
${Boost_LIBRARIES} 
${catkin_LIBRARIES} 
${ncurses_LIBRARIES})

target_link_libraries(${PROJECT_NAME}
${Boost_LIBRARIES} 
${catkin_LIBRARIES} 
${ncurses_LIBRARIES} 
${PROJECT_NAME}_lib)
