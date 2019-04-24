cmake_minimum_required(VERSION 2.8.3)
project(eurobot2019_collision_avoidance)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  eurobot2019_messages
  rospy
  std_msgs
)

catkin_package()

install(PROGRAMS
  scripts/my_python_script
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
