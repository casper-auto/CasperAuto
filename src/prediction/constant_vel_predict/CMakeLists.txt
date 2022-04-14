cmake_minimum_required(VERSION 3.0.2)
project(constant_vel_predict)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    casper_auto_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(LIBRARIES geometry_lib
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp casper_auto_msgs
)

# Declare a C++ library
add_library(geometry_lib
    src/geometry_utils.cpp
    src/frenet_frame.cpp
)

add_executable(constant_vel_predict_node
    src/constant_vel_predict_node.cpp
    src/constant_vel_predict.cpp
)

add_dependencies(constant_vel_predict_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(constant_vel_predict_node
    ${catkin_LIBRARIES}
    geometry_lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
