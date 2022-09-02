cmake_minimum_required(VERSION 3.0.2)
project(graph_optimizer)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

  tools	# tic_toc
)

set(LIBG2O "")
include(./cmake/g2o.cmake)

catkin_package( INCLUDE_DIRS include )
include_directories( include ${catkin_INCLUDE_DIRS})

include_directories( ${PROJECT_SOURCE_DIR}/include/graph_optimizer )

## Declare a C++ library
add_library(g2o_graph_optimizer
    src/g2o/g2o_graph_optimizer.cpp
)
target_include_directories( g2o_graph_optimizer  PUBLIC 
    include
) 
target_link_libraries(g2o_graph_optimizer PUBLIC
    ${LIBG2O}
)

