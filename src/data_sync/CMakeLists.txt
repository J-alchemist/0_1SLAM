cmake_minimum_required(VERSION 3.0.2)
project(data_sync)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  geometry_msgs
  sensor_msgs 

  sensor_type
  tools         #tf_convert data_publisher
  custom_msgs
)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

#一下两句，可以让其他包引用到此包的头文件
#xml注明外面的包
catkin_package( INCLUDE_DIRS include )
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/data_sync 
)

# message("---------------------")
# set(dir "${PROJECT_SOURCE_DIR}")
# message("${dir}")


#-----------lib------------------------
#---------------------------------
add_library(data_sync STATIC
        src/data_sync.cpp
)
target_include_directories(data_sync  PUBLIC
        include 
)
add_dependencies(data_sync imu cloud gnss tf_convert)
target_link_libraries(  data_sync
        imu
        cloud
        gnss 
        tf_convert
        data_publisher
)

#-----------ros node--------------------
#---------------------------------
add_executable(data_sync_node
        src/data_sync_node.cpp
)
add_dependencies(data_sync_node  ${catkin_EXPORTED_TARGETS}  data_sync)
target_link_libraries(data_sync_node  
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${EIGEN3_LIBRARIES}
      data_sync
)


