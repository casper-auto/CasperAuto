cmake_minimum_required(VERSION 3.0.2)
project(replay_path_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  casper_auto_msgs
  visualization_msgs
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

add_executable(replay_path_planner
  src/replay_path_planner.cpp
  src/replay_path_planner_node.cpp)

add_dependencies(replay_path_planner
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_executable(cruise_speed_pub
  src/cruise_speed_pub.cpp)

add_dependencies(cruise_speed_pub
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(replay_path_planner
 ${catkin_LIBRARIES}
)

target_link_libraries(cruise_speed_pub
 ${catkin_LIBRARIES}
)
