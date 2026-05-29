cmake_minimum_required(VERSION 2.8.3)
project(fpd_control)

## Trova i pacchetti necessari
find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

## Configura il supporto a setup.py
catkin_python_setup()

## Dichiarazione del pacchetto
catkin_package(
  CATKIN_DEPENDS duckietown_msgs geometry_msgs roscpp rospy std_msgs
)

## Includi directory
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Installa gli script Python (i nodi eseguibili)
catkin_install_python(PROGRAMS
  src/fpd_control/fpd_lane_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Installa anche la cartella config (con model.npz)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

