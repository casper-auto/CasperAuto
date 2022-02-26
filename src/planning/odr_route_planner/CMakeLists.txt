cmake_minimum_required(VERSION 3.0.2)
project(odr_route_planner)

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

add_executable(route_planner
  src/route_planner.cpp
  src/route_planner_node.cpp)

add_dependencies(route_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(route_planner
  ${catkin_LIBRARIES}
)
