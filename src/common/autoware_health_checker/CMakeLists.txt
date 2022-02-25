cmake_minimum_required(VERSION 2.8.3)
project(autoware_health_checker)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_system_msgs
  roscpp
  diagnostic_msgs
  jsk_rviz_plugins
  rosunit
  rosgraph_msgs
  ros_observer
  roslint
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES health_checker system_status_subscriber
  CATKIN_DEPENDS autoware_system_msgs rosgraph_msgs
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(HEALTH_CHECKER_SRC
  src/health_checker/health_checker.cpp
  src/health_checker/diag_buffer.cpp
  src/health_checker/rate_checker.cpp
  src/health_checker/value_manager.cpp
  src/health_checker/param_manager.cpp
)
add_library(health_checker
  ${HEALTH_CHECKER_SRC}
)
target_link_libraries(health_checker ${catkin_LIBRARIES})
add_dependencies(health_checker ${catkin_EXPORTED_TARGETS})

add_library(system_status_subscriber
  src/system_status_subscriber/system_status_subscriber.cpp
)
target_link_libraries(system_status_subscriber ${catkin_LIBRARIES})
add_dependencies(system_status_subscriber ${catkin_EXPORTED_TARGETS})

add_executable(health_aggregator
  src/health_aggregator/health_aggregator_node.cpp
  src/health_aggregator/health_aggregator.cpp
  src/health_aggregator/status_monitor.cpp
)
target_link_libraries(health_aggregator health_checker ${catkin_LIBRARIES})
add_dependencies(health_aggregator ${catkin_EXPORTED_TARGETS})

add_executable(health_analyzer
  src/health_analyzer/health_analyzer_node.cpp
  src/health_analyzer/health_analyzer.cpp
)
target_link_libraries(health_analyzer ${catkin_LIBRARIES})
add_dependencies(health_analyzer ${catkin_EXPORTED_TARGETS})

# CPP Execution programs
set(CPP_EXEC_NAMES health_aggregator health_analyzer)
foreach(cpp_exec_names ${CPP_EXEC_NAMES})
  install(TARGETS ${cpp_exec_names}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endforeach(cpp_exec_names)

# include header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install library
install(TARGETS health_checker system_status_subscriber
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test-autoware_health_checker
  test/test_autoware_health_checker.test
  test/src/test_autoware_health_checker.cpp
  ${HEALTH_CHECKER_SRC})
  target_link_libraries(test-autoware_health_checker
  ${catkin_LIBRARIES})

  roslint_add_test()
endif ()
