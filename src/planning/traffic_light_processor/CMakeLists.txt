cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_processor)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  nav_msgs
  casper_auto_msgs
  odr_map
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

add_executable(traffic_light_processor
  src/traffic_light_processor.cpp
  src/traffic_light_processor_node.cpp)

add_dependencies(traffic_light_processor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(traffic_light_processor
  ${catkin_LIBRARIES}
)
