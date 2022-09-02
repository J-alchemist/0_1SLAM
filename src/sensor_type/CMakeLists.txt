cmake_minimum_required(VERSION 3.0.2)
project(sensor_type)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

#该目录一定要是此cmakelists的子目录
add_subdirectory(GeographicLib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package( INCLUDE_DIRS  include )
include_directories( include   ${catkin_INCLUDE_DIRS} )

include_directories( 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/sensor_type
)

#-----------------------------------------------------------
# message("---------------------")
# set(dir "${PROJECT_SOURCE_DIR}")
# message("${dir}")
# set(dir "${PROJECT_BINARY_DIR}")
# message("${dir}")
# message("---------------------")
# 单独写的一个include: /opt/ros/kinetic/include
# message(${include})
# message("---------------------")
# 应该是xml相关
# message(${catkin_INCLUDE_DIRS})
# message("---------------------")
#-----------------------------------------------------------

# 设置库输出的路径 catkin工程好像设置没用
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# imu
add_library(imu STATIC
      src/imu.cpp
) 
target_include_directories(imu  PUBLIC
        include
        )
target_link_libraries( imu ${EIGEN3_LIBRARIES})
# cloud
add_library(cloud STATIC
      src/cloud.cpp
) 
target_include_directories(cloud  PUBLIC                
        include
        )       # 区别于include_directories， 只指定一个目标的头文件路径
target_link_libraries( cloud ${PCL_LIBRARIES})
# gnss
add_library(gnss STATIC
    src/gnss.cpp
) 
add_dependencies(gnss libGeographiccc)    #gnss依赖libGeographiccc，系统必须先编译libGeographiccc
target_include_directories(gnss PUBLIC 
        include  
        ${PROJECT_SOURCE_DIR}/GeographicLib/include/Geocentric 
) 
target_link_libraries( gnss libGeographiccc)

