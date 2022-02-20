cmake_minimum_required(VERSION 3.0.2)
project(route_reconstructor)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  nav_msgs
  casper_auto_msgs
  odr_map
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(route_reconstructor
  src/route_reconstructor.cpp
  src/route_reconstructor_node.cpp)

add_dependencies(route_reconstructor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(route_reconstructor
  ${catkin_LIBRARIES}
)
