cmake_minimum_required(VERSION 3.0.2)
project(carla_tf_light)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  sensor_msgs
)

add_service_files(
  DIRECTORY srv
  FILES GetNextTrafficLight.srv
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
  rospy
  message_runtime
  std_msgs
  geometry_msgs
)

catkin_install_python(PROGRAMS
  src/tf_light/tf_light.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
