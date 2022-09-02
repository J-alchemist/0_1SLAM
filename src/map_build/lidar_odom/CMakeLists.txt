cmake_minimum_required(VERSION 3.0.2)
project(lidar_odom)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf

  sensor_type
  custom_msgs
  tools		# registration_method filter_method file_manager
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package( INCLUDE_DIRS  include )
include_directories( include  ${catkin_INCLUDE_DIRS} )

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR} 
  ${PROJECT_SOURCE_DIR}/include/lidar_odom
)

set(DEPENDENCIRS_LIBRARIES "")
list(APPEND DEPENDENCIRS_LIBRARIES 
      registration_method 
      filter_method   
      file_manager 
      internal_pose 
      data_publisher
      gnss
      cloud
      imu
)

add_library(lidar_odom STATIC
    src/lidar_odom.cpp
) 
target_include_directories(lidar_odom  PUBLIC
    include
)
add_dependencies(lidar_odom ${DEPENDENCIRS_LIBRARIES} ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(lidar_odom PUBLIC
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${YAML_CPP_LIBRARIES} 
    
    ${DEPENDENCIRS_LIBRARIES}
)


####################
## Build ROS NODE ##
####################
add_executable(lidar_odom_node  src/lidar_odom_node.cpp)
add_dependencies(lidar_odom_node  ${catkin_EXPORTED_TARGETS}  lidar_odom)
target_link_libraries(lidar_odom_node
    # ${DEPENDENCIRS_LIBRARIES} 
    lidar_odom
)




