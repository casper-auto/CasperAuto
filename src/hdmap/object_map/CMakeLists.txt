cmake_minimum_required(VERSION 2.8.12)

project(object_map)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  lanelet2_extension
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  roslint
  sensor_msgs
  tf
  vector_map
)

# TODO add other files into roslint
set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp(
  nodes/wayarea2grid_lanelet2/wayarea2grid_lanelet2_node.cpp
  nodes/wayarea2grid_lanelet2/wayarea2grid_lanelet2.cpp
  include/wayarea2grid_lanelet2/wayarea2grid_lanelet2.h
)

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_map_utils_lib
  CATKIN_DEPENDS
    pcl_ros
    pcl_conversions
    tf
    sensor_msgs
    nav_msgs
    autoware_msgs
    grid_map_msgs
    vector_map
    lanelet2_extension
)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
### object_map_utils ###
add_library(object_map_utils_lib
  include/object_map/object_map_utils.cpp
  include/object_map/object_map_utils.hpp
)
target_link_libraries(object_map_utils_lib
  ${catkin_LIBRARIES}
)

### laserscan2costmap ###
add_executable(laserscan2costmap
  nodes/laserscan2costmap/laserscan2costmap.cpp
)

target_link_libraries(laserscan2costmap
  ${catkin_LIBRARIES}
)


### potential_field ###
add_executable(potential_field
  nodes/potential_field/potential_field.cpp
)
add_dependencies(potential_field
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(potential_field
  ${catkin_LIBRARIES}
)

### grid_map_filter ###
add_library(grid_map_filter_lib
  nodes/grid_map_filter/grid_map_filter.h
  include/object_map/object_map_utils.hpp
  nodes/grid_map_filter/grid_map_filter.cpp
)
target_link_libraries(grid_map_filter_lib
  ${catkin_LIBRARIES}
  object_map_utils_lib
)
add_executable(grid_map_filter
  nodes/grid_map_filter/grid_map_filter_node.cpp
  nodes/grid_map_filter/grid_map_filter.h
)
target_link_libraries(grid_map_filter
  ${catkin_LIBRARIES}
  object_map_utils_lib
  grid_map_filter_lib
)

### wayarea2grid ###
add_library(wayarea2grid_lib
  nodes/wayarea2grid/wayarea2grid.h
  include/object_map/object_map_utils.hpp
  nodes/wayarea2grid/wayarea2grid.cpp
)
target_link_libraries(wayarea2grid_lib
  ${catkin_LIBRARIES}
  object_map_utils_lib
)

add_executable(wayarea2grid
  nodes/wayarea2grid/wayarea2grid.h
  nodes/wayarea2grid/wayarea2grid_node.cpp
)

target_link_libraries(wayarea2grid
  ${catkin_LIBRARIES}
  wayarea2grid_lib
)

add_library(wayarea2grid_lanelet2_lib
  include/object_map/object_map_utils.hpp
  nodes/wayarea2grid_lanelet2/wayarea2grid_lanelet2.cpp
)
target_link_libraries(wayarea2grid_lanelet2_lib
  ${catkin_LIBRARIES}
  object_map_utils_lib
)

add_executable(wayarea2grid_lanelet2
  nodes/wayarea2grid_lanelet2/wayarea2grid_lanelet2_node.cpp
)

target_link_libraries(wayarea2grid_lanelet2
  ${catkin_LIBRARIES}
  wayarea2grid_lanelet2_lib
)

install(
  TARGETS
    wayarea2grid
    wayarea2grid_lib
    wayarea2grid_lanelet2
    wayarea2grid_lanelet2_lib
    grid_map_filter_lib
    grid_map_filter
    potential_field
    laserscan2costmap
    object_map_utils_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

if (CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
