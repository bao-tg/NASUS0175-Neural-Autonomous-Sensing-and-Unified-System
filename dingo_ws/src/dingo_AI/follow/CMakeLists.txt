cmake_minimum_required(VERSION 3.0.2)
project(follow)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/person_follow_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
