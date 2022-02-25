cmake_minimum_required(VERSION 2.8.3)
project(ros_observer)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  roslint
)

find_package(Boost REQUIRED COMPONENTS
  thread
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lib_ros_observer
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ros_observer
  src/ros_observer.cpp
)

target_link_libraries(
  ros_observer
  rt ${Boost_LIBRARIES}
  pthread
)

add_dependencies(
  ros_observer
  ${catkin_EXPORTED_TARGETS}
)

add_library(
  lib_ros_observer
  lib/lib_ros_observer.cpp
)

target_link_libraries(
  lib_ros_observer
  ${catkin_LIBRARIES}
  rt
  ${Boost_LIBRARIES}
)

add_dependencies(
  lib_ros_observer
  ${catkin_EXPORTED_TARGETS}
)

# include header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install library
install(TARGETS ros_observer lib_ros_observer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (CATKIN_ENABLE_TESTING)
  roslint_add_test()
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_ros_observer
    test/test_ros_observer.test
    test/src/test_ros_observer.cpp
  )
  target_link_libraries(test_ros_observer
    lib_ros_observer
    ${catkin_LIBRARIES}
  )
endif()
