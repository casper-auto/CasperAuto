cmake_minimum_required(VERSION 2.8.3)
project(driveway_object_filter)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  casper_auto_msgs
  geometry_msgs
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  roscpp
  sensor_msgs
  tf
  odr_map
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# node
add_executable(driveway_object_filter
  src/driveway_object_filter_node.cpp
  src/driveway_object_filter.cpp
)

target_link_libraries(driveway_object_filter
  ${catkin_LIBRARIES}
)

add_dependencies(driveway_object_filter
  ${catkin_EXPORTED_TARGETS}
)
