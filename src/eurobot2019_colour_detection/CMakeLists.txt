cmake_minimum_required(VERSION 2.8.3)
project(eurobot2019_colour_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2
        tf2_ros
        image_transport
        std_msgs
        geometry_msgs
        sensor_msgs
        cv_bridge
)


## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node
        colour_detection.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRABRIES}
        ${OpenCV_LIBS}
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY objects
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY camera
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

