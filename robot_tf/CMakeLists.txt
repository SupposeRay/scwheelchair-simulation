cmake_minimum_required(VERSION 2.8.3)
project(robot_tf)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package(

)

include_directories(
 include  ${catkin_INCLUDE_DIRS}
)

#############
## Scripts ##
#############

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})


add_executable(turtlebot_tf_broadcaster src/turtlebot_tf_broadcaster.cpp)
target_link_libraries(turtlebot_tf_broadcaster ${catkin_LIBRARIES})

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})
