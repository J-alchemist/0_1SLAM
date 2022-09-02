cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY 
    msg/
    
  FILES
    poseWithIndex.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY 
    srv/

  FILES
    saveMap.srv
    optimizeMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime)


