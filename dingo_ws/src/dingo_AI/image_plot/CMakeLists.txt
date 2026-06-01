cmake_minimum_required(VERSION 3.0.2)
project(image_plot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/image_plot_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
