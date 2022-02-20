cmake_minimum_required(VERSION 2.8.3)
project(vector_map_server)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  message_generation
  roscpp
  roslint
  vector_map
  vector_map_msgs
  visualization_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

add_service_files(
  FILES
  GetCrossRoad.srv
  GetCrossWalk.srv
  GetCurb.srv
  GetCurveMirror.srv
  GetDTLane.srv
  GetDriveOnPortion.srv
  GetFence.srv
  GetGuardRail.srv
  GetGutter.srv
  GetLane.srv
  GetNode.srv
  GetRailCrossing.srv
  GetRoadEdge.srv
  GetRoadMark.srv
  GetRoadPole.srv
  GetRoadSign.srv
  GetSideStrip.srv
  GetSideWalk.srv
  GetSignal.srv
  GetStopLine.srv
  GetStreetLight.srv
  GetUtilityPole.srv
  GetWall.srv
  GetWayArea.srv
  GetWhiteLine.srv
  GetZebraZone.srv
  PositionState.srv
)

generate_messages(
  DEPENDENCIES
    autoware_msgs
    geometry_msgs
    vector_map_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    autoware_msgs
    vector_map_msgs
    vector_map
    visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  nodes/vector_map_server/vector_map_server.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(vector_map_client
  nodes/vector_map_client/vector_map_client.cpp
)

target_link_libraries(vector_map_client
  ${catkin_LIBRARIES}
)

add_dependencies(vector_map_client
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++14,-runtime/references,-whitespace/braces")
roslint_cpp()

install(TARGETS ${PROJECT_NAME} vector_map_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
