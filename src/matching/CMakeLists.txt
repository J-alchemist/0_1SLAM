cmake_minimum_required(VERSION 3.0.2)
project(matching)

add_compile_options(-std=c++11)
# set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  geometry_msgs
  sensor_msgs 

  sensor_type
  tools   # lib: registration_method filter_method tf_convert  data_publisher
  custom_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package( INCLUDE_DIRS include )   
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
  ${PROJECT_SOURCE_DIR}/include/matching
)

set(ALL_TARGET_LIBRARIES "")
list(APPEND ALL_TARGET_LIBRARIES 
      registration_method
      filter_method
      tf_convert
      data_publisher
      gnss
      imu
      cloud
)

# #---------------------------
# #---------------------------
add_library(matching STATIC
    src/matching.cpp
) 
target_include_directories(matching  PUBLIC
    include
)
add_dependencies(matching   ${ALL_TARGET_LIBRARIES} )
target_link_libraries(matching
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${YAML_CPP_LIBRARIES} 

    ${ALL_TARGET_LIBRARIES}
)

#-----------ros node----------------
#-----------------------------------
add_executable(matching_node src/matching_node.cpp)
add_dependencies(matching_node  ${catkin_EXPORTED_TARGETS} matching)
target_link_libraries(matching_node 
    matching
)

# 库连接具有传递性
