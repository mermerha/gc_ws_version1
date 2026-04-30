cmake_minimum_required(VERSION 3.0.2)
project(opencv_detect)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

add_service_files(
  FILES
  Color_detection.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs sensor_msgs cv_bridge
)

catkin_install_python(PROGRAMS
  scripts/color_detect_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
