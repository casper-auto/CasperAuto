cmake_minimum_required(VERSION 2.8.3)
project(autoware_system_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rosgraph_msgs
  std_msgs
)

add_message_files(
  FILES
    DiagnosticStatus.msg
    DiagnosticStatusArray.msg
    HardwareStatus.msg
    NodeStatus.msg
    SystemStatus.msg
)

generate_messages(DEPENDENCIES
  rosgraph_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    rosgraph_msgs
    std_msgs
)
