cmake_minimum_required(VERSION 3.0.2)
project(sensor_type)

add_compile_options(-std=c++11)

#该目录一定要是此cmakelists的子目录
add_subdirectory(GeographicLib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(PCL 1.7 REQUIRED)

catkin_package( INCLUDE_DIRS  include )
include_directories( include   ${catkin_INCLUDE_DIRS} )

include_directories( 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include/sensor_type
)

add_library(imu STATIC
      src/imu.cpp
) 
target_include_directories(imu  PUBLIC
        include
        )

add_library(cloud STATIC
      src/cloud.cpp
) 
target_include_directories(cloud  PUBLIC
        include
        )

add_library(gnss STATIC
    src/gnss.cpp
) 
target_link_libraries(gnss  PUBLIC
          libGeographiccc
        )
target_include_directories(gnss PUBLIC
    ${catkin_INCLUDE_DIRS}
) 

add_library(tf_convert STATIC
    src/tf_convert.cpp
) 
target_include_directories(tf_convert  PUBLIC
        include ${EIGEN3_INCLUDE_DIR}
        )
