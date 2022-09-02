cmake_minimum_required(VERSION 3.0.2)
project(loop_closing)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf

  sensor_type
  tools       # registration_method filter_method
  custom_msgs
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
  ${PROJECT_SOURCE_DIR}/include/loop_closing
)

set(DEPENDENCIRS_LIBRARIES "")
list(APPEND DEPENDENCIRS_LIBRARIES 
      registration_method 
      filter_method  
      data_publisher
      imu
      gnss
      cloud   
)

#----------------------------------
add_library(loop_closing STATIC
    src/loop_closing.cpp
) 
target_include_directories(loop_closing  PUBLIC
    include
)
add_dependencies(loop_closing ${DEPENDENCIRS_LIBRARIES})
target_link_libraries(loop_closing PUBLIC
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
 
    ${DEPENDENCIRS_LIBRARIES}
)


####################
## Build ROS NODE ##
####################
add_executable(loop_closing_node  src/loop_closing_node.cpp)
add_dependencies(loop_closing_node  ${catkin_EXPORTED_TARGETS}  lidar_odom)
target_link_libraries(loop_closing_node
    loop_closing
)

