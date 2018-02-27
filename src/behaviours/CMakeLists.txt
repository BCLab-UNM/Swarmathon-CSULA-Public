cmake_minimum_required(VERSION 2.8.3)
project(behaviours)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  random_numbers
  tf
  apriltags_ros
  swarmie_msgs
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs swarmie_msgs roscpp sensor_msgs std_msgs random_numbers tf apriltags_ros swarmie_msgs grid_map_core grid_map_ros grid_map_cv grid_map_filters grid_map_loader grid_map_msgs grid_map_octomap grid_map_rviz_plugin grid_map_visualization
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  behaviours
  src/Tag.cpp
  src/ObstacleController.cpp 
  src/PickUpController.cpp
  src/DropOffController.cpp
  src/SearchController.cpp
  src/ROSAdapter.cpp
  src/PID.cpp
  src/DriveController.cpp
  src/RangeController.cpp
  src/LogicController.cpp
  src/ManualWaypointController.cpp
  src/GridtoZone.cpp
)

add_dependencies(behaviours ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  behaviours
  ${catkin_LIBRARIES}
)

