cmake_minimum_required(VERSION 3.0.2)
project(claw_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/claw_service_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
