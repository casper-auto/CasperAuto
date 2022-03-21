cmake_minimum_required(VERSION 2.8.3)

project(op_ros_helpers)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  jsk_recognition_msgs
  libwaypoint_follower
  map_file
  op_planner
  op_simu
  op_utility
  pcl_conversions
  pcl_ros
  sensor_msgs
  tf
  vector_map_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    autoware_msgs
    geometry_msgs
    libwaypoint_follower
    vector_map_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(ROS_HELPERS_SRC
  src/PolygonGenerator.cpp
  src/op_ROSHelpers.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${ROS_HELPERS_SRC}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
