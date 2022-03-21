cmake_minimum_required(VERSION 2.8.3)
project(op_utility)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  vector_map_msgs
  vector_map_server
)

find_package(TinyXML REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS vector_map_msgs vector_map_server
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(UTILITYH_SRC
  src/DataRW.cpp
  src/UtilityH.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${UTILITYH_SRC}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test-op_utility
    test/test_op_utility.test
    test/src/test_op_utility.cpp
  )
  add_dependencies(test-op_utility ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test-op_utility ${catkin_LIBRARIES} ${PROJECT_NAME})
endif ()
