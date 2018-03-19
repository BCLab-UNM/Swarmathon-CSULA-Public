cmake_minimum_required(VERSION 2.8.3)
project(grid_map_swarm)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  swarmie_msgs
  cv_bridge
  octomap_msgs
  filters
)

catkin_package(
  CATKIN_DEPENDS grid_map_core grid_map_ros grid_map_cv grid_map_filters grid_map_loader grid_map_msgs grid_map_octomap grid_map_rviz_plugin grid_map_visualization geometry_msgs roscpp sensor_msgs std_msgs tf swarmie_msgs cv_bridge octomap_msgs filters
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  grid_map_swarm
  src/grid_swarm.cpp
)

add_dependencies(grid_map_swarm ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  grid_map_swarm
  ${catkin_LIBRARIES}
)

