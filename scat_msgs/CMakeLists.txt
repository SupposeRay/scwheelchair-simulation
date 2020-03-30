cmake_minimum_required(VERSION 2.8.3)
project(scat_msgs)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  std_msgs
)

 add_message_files(
   FILES
   EnvObject.msg
   SharedVelocity.msg
   EnvObjectList.msg
 )

add_service_files(
  FILES
  SetMode.srv
)

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
  geometry_msgs   
  std_msgs 
 )


catkin_package(
  CATKIN_DEPENDS message_runtime  geometry_msgs std_msgs
)
