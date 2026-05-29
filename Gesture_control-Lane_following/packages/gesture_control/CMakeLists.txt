cmake_minimum_required(VERSION 2.8.3)
project(gesture_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  duckietown_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/tcp_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
