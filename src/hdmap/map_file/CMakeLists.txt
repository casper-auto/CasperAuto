cmake_minimum_required(VERSION 2.8.3)

project(map_file)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  lanelet2_extension
  pcl_ros
  roscpp
  std_msgs
  tf
  tf2_geometry_msgs
  tf2_ros
  vector_map
  visualization_msgs
)

find_package(PCL REQUIRED COMPONENTS io)
find_package(Boost REQUIRED COMPONENTS filesystem)

# See: https://github.com/ros-perception/perception_pcl/blob/lunar-devel/pcl_ros/CMakeLists.txt#L10-L22
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # FIXME: this causes duplicates and not found error in ubuntu:zesty
  list(REMOVE_ITEM PCL_LIBRARIES "/usr/lib/libmpi.so")

  # For debian: https://github.com/ros-perception/perception_pcl/issues/139
  list(REMOVE_ITEM PCL_IO_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt"
  )
endif()

find_package(CURL REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES get_file
  CATKIN_DEPENDS
    autoware_msgs
    geometry_msgs
    std_msgs
    tf2_geometry_msgs
    visualization_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_IO_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_library(get_file
  lib/map_file/get_file.cpp
)
target_link_libraries(get_file ${CURL_LIBRARIES})

add_executable(points_map_loader nodes/points_map_loader/points_map_loader.cpp)
target_link_libraries(points_map_loader ${catkin_LIBRARIES} get_file ${CURL_LIBRARIES} ${PCL_IO_LIBRARIES})
add_dependencies(points_map_loader
  ${catkin_EXPORTED_TARGETS}
)

add_executable(vector_map_loader nodes/vector_map_loader/vector_map_loader.cpp)
target_link_libraries(vector_map_loader ${catkin_LIBRARIES} ${vector_map_LIBRARIES} get_file ${CURL_LIBRARIES})
add_dependencies(vector_map_loader ${catkin_EXPORTED_TARGETS})

add_executable(lanelet2_map_loader nodes/lanelet2_map_loader/lanelet2_map_loader.cpp)
target_link_libraries(lanelet2_map_loader ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(lanelet2_map_loader ${catkin_EXPORTED_TARGETS})

add_executable(lanelet2_map_visualization nodes/lanelet2_map_loader/lanelet2_map_visualization.cpp)
target_link_libraries(lanelet2_map_visualization ${catkin_LIBRARIES})
add_dependencies(lanelet2_map_visualization ${catkin_EXPORTED_TARGETS})

add_executable(points_map_filter nodes/points_map_filter/points_map_filter_node.cpp nodes/points_map_filter/points_map_filter.cpp)
target_link_libraries(points_map_filter ${catkin_LIBRARIES})
add_dependencies(points_map_filter ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(
  TARGETS
    get_file
    points_map_loader
    vector_map_loader
    lanelet2_map_loader
    lanelet2_map_visualization
    points_map_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
