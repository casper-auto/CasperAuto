cmake_minimum_required(VERSION 3.0.2)
project(lidar_detector)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  derived_object_msgs
  visualization_msgs
)

find_package(PCL 1.2 REQUIRED)

catkin_package()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(lidar_detector
  src/lidar_detector_node.cpp
  src/lidar_detector.cpp
)

target_link_libraries(lidar_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(lidar_detector
  ${catkin_EXPORTED_TARGETS}
)
