cmake_minimum_required(VERSION 3.0.2)
project(trajectory_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  casper_auto_msgs
  carla_msgs
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

add_executable(trajectory_controller
  src/trajectory_controller.cpp
  src/trajectory_controller_node.cpp)

target_link_libraries(trajectory_controller
 ${catkin_LIBRARIES}
)

add_dependencies(trajectory_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
