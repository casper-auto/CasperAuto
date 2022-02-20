cmake_minimum_required(VERSION 3.0.2)
project(odr_map_loader)

# add_compile_options(-std=c++11)
add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  casper_auto_msgs
  visualization_msgs
  jsk_recognition_msgs
  casper_auto_msgs
  odr_map
)

find_package(XMLRPC REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# odr_map_loader
add_executable(odr_map_loader
  src/odr_map_loader.cpp
)

target_link_libraries(odr_map_loader
  ${catkin_LIBRARIES}
)

# odr_map_path_finder
add_executable(odr_map_path_finder
  src/odr_map_path_finder.cpp
)

target_link_libraries(odr_map_path_finder
  ${catkin_LIBRARIES}
)

# traffic_info_loader
add_executable(traffic_info_loader
  src/traffic_info_loader.cpp
)

target_link_libraries(traffic_info_loader
  ${catkin_LIBRARIES}
)
