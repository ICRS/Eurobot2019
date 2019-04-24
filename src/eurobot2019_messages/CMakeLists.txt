cmake_minimum_required(VERSION 2.8.3)
project(eurobot2019_messages)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  drop_motors.msg
  grabber_motors.msg
  pickup.msg
  grabber_motors.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
