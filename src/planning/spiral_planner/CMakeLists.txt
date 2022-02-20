cmake_minimum_required(VERSION 3.0.2)
project(spiral_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  casper_auto_msgs
  visualization_msgs
  derived_object_msgs
  jsk_recognition_msgs
  autoware_msgs
)

#find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

include_directories(
  include
  #${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(spiral_planner
  src/spiral_planner_node.cpp
  src/frenet_frame.cpp
  src/spiral_planner/spiral_planner.cpp
  src/path_optimizer/meta.cpp
  src/path_optimizer/path_optimizer.cpp
  src/collision_checker/collision_checker.cpp
  src/velocity_planner/velocity_planner.cpp
)

target_link_libraries(spiral_planner
  ${catkin_LIBRARIES}
  # ${EIGEN3_LIBRARIES}
)
