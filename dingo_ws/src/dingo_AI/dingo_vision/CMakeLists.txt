cmake_minimum_required(VERSION 3.0.2)
project(dingo_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ros_gst_camera.py
  scripts/ros_image_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/run_tracker.sh
  scripts/run_ros_gst_camera.sh
  scripts/run_ros_image_tracker.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
