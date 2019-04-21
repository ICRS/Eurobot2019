cmake_minimum_required(VERSION 2.8.3)
project(eurobot2019_tf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS rospy tf2 tf2_ros
)

install(PROGRAMS
  scripts/broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
