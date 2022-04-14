cmake_minimum_required(VERSION 3.0.2)
project(speed_profile_planner)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  nav_msgs
  casper_auto_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

find_package(PythonLibs REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PYTHON_INCLUDE_DIRS}
)

#############################################################################
# Declare Library                                                           #
#############################################################################

file(GLOB SOURCES "src/speed_planner_lib/*.cpp")

add_library(speed_planner_lib SHARED ${SOURCES})

target_include_directories(speed_planner_lib PUBLIC
  include
  ${PYTHON_INCLUDE_DIRS}
)

target_link_libraries(speed_planner_lib
  ipopt
  ${PYTHON_LIBRARIES}
)

#############################################################################
# Declare Executable                                                        #
#############################################################################

add_executable(speed_profile_planner
  src/speed_profile_planner.cpp)

add_dependencies(speed_profile_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(speed_profile_planner
  ${catkin_LIBRARIES}
  speed_planner_lib
)

#############################################################################
# Unit Test                                                                 #
#############################################################################

# add_executable(obstacle_test test/src/obstacle_test.cpp)
# target_link_libraries(obstacle_test speed_planner_lib)

# add_executable(st_graph_test test/src/st_graph_test.cpp)
# target_link_libraries(st_graph_test speed_planner_lib)

# add_executable(st_cell_planner_test test/src/st_cell_planner_test.cpp)
# target_link_libraries(st_cell_planner_test speed_planner_lib)

# add_executable(speed_planner_test test/src/speed_planner_test.cpp)
# target_link_libraries(speed_planner_test speed_planner_lib)
