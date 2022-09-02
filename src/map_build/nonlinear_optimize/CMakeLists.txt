cmake_minimum_required(VERSION 3.0.2)
project(nonlinear_optimize)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf

  sensor_type
  custom_msgs
  tools
  graph_optimizer
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
  ${PROJECT_SOURCE_DIR}/include/nonlinear_optimize
)

set(DEPENDENCIRS_LIBRARIES "")
list(APPEND DEPENDENCIRS_LIBRARIES 
      registration_method 
      filter_method   
      file_manager  
      g2o_graph_optimizer
      internal_pose
      data_publisher
      gnss
      cloud
      imu
)

# nonlinear_optimize
add_library(nonlinear_optimize STATIC
    src/nonlinear_optimize.cpp
) 
target_include_directories(nonlinear_optimize  PUBLIC
    include
)
add_dependencies(nonlinear_optimize ${DEPENDENCIRS_LIBRARIES} ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(nonlinear_optimize PUBLIC
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${YAML_CPP_LIBRARIES} 

    ${DEPENDENCIRS_LIBRARIES}
)
# rebuild_map
add_library(rebuild_map STATIC
    src/rebuild_map.cpp
) 
target_include_directories(rebuild_map  PUBLIC
    include
)
add_dependencies(rebuild_map ${DEPENDENCIRS_LIBRARIES} ${custom_msgs_EXPORTED_TARGETS})
target_link_libraries(rebuild_map PUBLIC
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${YAML_CPP_LIBRARIES} 

    ${DEPENDENCIRS_LIBRARIES}
)

####################
## Build ROS NODE ##
####################
add_executable(nonlinear_optimize_node  src/nonlinear_optimize_node.cpp)
add_dependencies(nonlinear_optimize_node  ${catkin_EXPORTED_TARGETS}  nonlinear_optimize)
target_link_libraries(nonlinear_optimize_node
   nonlinear_optimize
)

add_executable(rebuild_map_node  src/rebuild_map_node.cpp)
add_dependencies(rebuild_map_node  ${catkin_EXPORTED_TARGETS}  rebuild_map)
target_link_libraries(rebuild_map_node
    rebuild_map
)

####################
##     离线配准    ##
####################
add_executable(offline_compose_map  src/offline_compose_map.cpp )
add_dependencies(offline_compose_map   cloud  filter_method )
target_link_libraries(offline_compose_map
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    filter_method
    cloud
)   


