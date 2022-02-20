cmake_minimum_required(VERSION 3.0.2)
project(vehicle_description)

find_package(catkin REQUIRED)

catkin_package()

# Uncompress meshes for individual vehicle models
MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()
SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR}/meshes)
FOREACH(urdf_model ${SUBDIRS})
  FILE(GLOB_RECURSE files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/meshes/${urdf_model}" "${CMAKE_CURRENT_SOURCE_DIR}/meshes/${urdf_model}/*.tar.gz")
  FOREACH(filename ${files})
    MESSAGE(STATUS "Extracting file: ${filename}")
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E tar xzf ${filename}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/meshes/${urdf_model}
    )
  ENDFOREACH(filename)
ENDFOREACH(urdf_model)

# Uncompress meshes in root of meshes folder
FILE(GLOB files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/meshes" "${CMAKE_CURRENT_SOURCE_DIR}/meshes/*.tar.gz")
FOREACH(filename ${files})
  MESSAGE(STATUS "Extracting file: ${filename}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf ${filename}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/meshes
  )
ENDFOREACH(filename)

install(DIRECTORY meshes urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN *.tar.gz EXCLUDE
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
