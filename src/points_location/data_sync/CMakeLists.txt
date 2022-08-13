cmake_minimum_required(VERSION 3.0.2)
project(data_sync)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  geometry_msgs
  sensor_msgs 
  sensor_type
)
find_package(PCL 1.7 REQUIRED)

#一下两句，可以让其他包引用到此包的头文件
catkin_package( INCLUDE_DIRS include )
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include/data_sync 
)

add_library(data_sync STATIC
  src/data_sync.cpp
)
target_include_directories(data_sync  PUBLIC
        include
        )

target_link_libraries(data_sync PUBLIC
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
    gnss 
    imu
  )

# target_include_directories(data_sync PUBLIC
#         ${catkin_INCLUDE_DIRS}
#         ) 
