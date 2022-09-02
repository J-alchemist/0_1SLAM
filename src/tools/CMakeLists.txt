cmake_minimum_required(VERSION 3.0.2)
project(tools)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  nav_msgs
  geometry_msgs
  sensor_msgs 

  sensor_type       #cloud
  custom_msgs
)
find_package(Boost REQUIRED COMPONENTS system) 
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

include(./cmake/global_defination.cmake)

catkin_package( INCLUDE_DIRS include )
include_directories( include ${catkin_INCLUDE_DIRS})

include_directories(
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
  ${PROJECT_SOURCE_DIR}/include/tools
)

set(ALL_TARGET_LIBRARIES "")
list(APPEND  ALL_TARGET_LIBRARIES 
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${EIGEN3_LIBRARIES}
      ${YAML_CPP_LIBRARIES}
)

# 编译方式一
# file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
# list(REMOVE_ITEM ALL_SRCS "src/*_node.cpp")
# add_executable(matching_node src/matching_node.cpp ${ALL_SRCS})
# add_dependencies(matching_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(matching_node ${ALL_TARGET_LIBRARIES})


# 编译方式二
#------------文件---------------
add_library( file_manager STATIC
    src/file_manager.cpp
)
target_include_directories(file_manager  PUBLIC
    include 
)
target_link_libraries(file_manager  ${Boost_LIBRARIES} )
#------------匹配---------------
add_library(registration_method STATIC
    src/ndt_registration.cpp 
    src/icp_registration.cpp 
) 
target_include_directories(registration_method  PUBLIC
    include 
)
target_link_libraries(registration_method  ${ALL_TARGET_LIBRARIES})
#------------滤波---------------
add_library(filter_method STATIC
    src/voxel_filter.cpp
    src/box_filter.cpp
) 
target_include_directories(filter_method  PUBLIC
    include
)
target_link_libraries(filter_method  ${ALL_TARGET_LIBRARIES})
#------------TF---------------
add_library(tf_convert STATIC
    src/tf_convert.cpp
) 
target_include_directories(tf_convert  PUBLIC
    include
)
target_link_libraries(tf_convert  ${ALL_TARGET_LIBRARIES})
#-------------内部数据类型---------------
add_library(internal_pose STATIC
    src/internal_pose.cpp
) 
add_dependencies(internal_pose   cloud)
target_include_directories(internal_pose  PUBLIC
    include
)
target_link_libraries(internal_pose  ${EIGEN3_LIBRARIES} cloud)
#-------------常用数据发布---------------
add_library(data_publisher STATIC
    src/data_publisher.cpp
) 
add_dependencies(data_publisher   internal_pose  cloud  ${custom_msgs_EXPORTED_TARGETS})
target_include_directories(data_publisher  PUBLIC
    include
)
target_link_libraries(data_publisher  
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    internal_pose
    cloud
)




