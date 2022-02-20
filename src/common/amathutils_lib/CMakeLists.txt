cmake_minimum_required(VERSION 2.8.11)
project(amathutils_lib) # autoware math utility

find_package(Eigen3 REQUIRED)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  roslint
  tf
  tf2
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES amathutils_lib
  CATKIN_DEPENDS
    autoware_msgs
    tf
    tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(amathutils_lib
  src/Amathutils.cpp
  src/kalman_filter.cpp
  src/time_delay_kalman_filter.cpp
  src/butterworth_filter.cpp
)
target_link_libraries(amathutils_lib
  ${catkin_LIBRARIES}
)

add_dependencies(amathutils_lib
  ${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS amathutils_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB_RECURSE ROSLINT_FILES
  LIST_DIRECTORIES false
  *.cpp *.h *.hpp
)

list(APPEND ROSLINT_CPP_OPTS "--extensions=cc,h,hpp,cpp,cu,cuh" "--filter=-build/c++11,-runtime/references")
roslint_cpp(${ROSLINT_FILES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(amathutils-test test/test_amathutils_lib.test test/src/test_amathutils_lib.cpp)
  target_link_libraries(amathutils-test ${catkin_LIBRARIES} amathutils_lib)
  add_rostest_gtest(test-kalman_filter
    test/test_kalman_filter.test
    test/src/test_kalman_filter.cpp
    src/kalman_filter.cpp
    src/time_delay_kalman_filter.cpp
  )
  target_link_libraries(test-kalman_filter ${catkin_LIBRARIES})

  add_rostest_gtest(test-butterworth_filter
    test/test_butterworth_filter.test
    test/src/test_butterworth_filter.cpp
    src/butterworth_filter.cpp
  )
  target_link_libraries(test-butterworth_filter ${catkin_LIBRARIES})
  roslint_add_test()
endif()
