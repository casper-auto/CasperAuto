cmake_minimum_required(VERSION 3.0.2)
project(odr_map)

add_compile_options(-std=c++14 -O3)
# add_compile_options(-Wall)

## find arch
execute_process(COMMAND
  dpkg-architecture
  -qDEB_HOST_ARCH
  OUTPUT_VARIABLE
  CMAKE_DEB_HOST_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

MESSAGE(STATUS "Using CMAKE_DEB_HOST_ARCH: ${CMAKE_DEB_HOST_ARCH}")

set(EXTRALIB_DIR ${PROJECT_SOURCE_DIR}/include/libcarla-install/lib)
MESSAGE(STATUS "  EXTRALIB_DIR = ${EXTRALIB_DIR}")

set(LIBCARLA_CLIENT libcarla_client.a)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf
  visualization_msgs
  rviz
  pluginlib
  sensor_msgs
  uuid_msgs
  casper_auto_msgs
)

add_service_files(
  FILES
  GenerateWaypoints.srv
  GetMapObject.srv
  GetMapGeoLocation.srv
  GetMapPolygons.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  uuid_msgs
  casper_auto_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    include/libcarla-install/include
    include/libcarla-install/include/system
  LIBRARIES odr_map
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
  #  DEPENDS system_lib
)

###########
## Build ##
###########

#Qt5 configulation
set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/libcarla-install/include
  ${PROJECT_SOURCE_DIR}/include/libcarla-install/include/system
  ${catkin_INCLUDE_DIRS}
)

add_library(odr_map SHARED src/odr_map.cpp src/PathListParser.cpp)

target_link_libraries(odr_map
  ${catkin_LIBRARIES}
  ${EXTRALIB_DIR}/libboost_filesystem.a
  ${EXTRALIB_DIR}/libboost_python27.a
  ${EXTRALIB_DIR}/${LIBCARLA_CLIENT}
  ${EXTRALIB_DIR}/librpc.a
)

target_include_directories(odr_map PUBLIC ${catkin_INCLUDE_DIRS})

add_dependencies(odr_map
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(odr_map_server src/odr_map_server.cpp)

target_link_libraries(odr_map_server
  odr_map
  ${catkin_LIBRARIES}
  ${EXTRALIB_DIR}/libboost_filesystem.a
  ${EXTRALIB_DIR}/libboost_python27.a
  ${EXTRALIB_DIR}/${LIBCARLA_CLIENT}
  ${EXTRALIB_DIR}/librpc.a
)

target_include_directories(odr_map_server PUBLIC ${catkin_INCLUDE_DIRS})

add_dependencies(odr_map_server
  odr_map
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
