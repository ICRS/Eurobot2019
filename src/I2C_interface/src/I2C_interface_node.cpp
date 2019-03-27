#include <ros/ros.h>

#include "message_interface.hpp"
#include <string>
#include <sstream>

int main(int argc, char **argv) {
    // Init ROS
    ros::init(argc, argv, "I2C_interface_module");

    // Get node handle
    ros::NodeHandle node_handle;

    MessageInterface<std_msgs::Empty, std_msgs::Empty>
                drop_interface(100, "drop_status",
                                  10, "drop_motors");

    // Create Localisaion interface
    MessageInterface<std_msgs::Empty, std_msgs::Empty>
                grabber_interface(100, "grabber_status",
                                  10, "grabber_motors");

    // check names of channels
    MessageInterface<std_msgs::Empty, std_msgs::Empty>
                navigation_interface(100, "odometry",
                                  10, "cmd_vel");


    // Control loop rate to be 100 Hz
    ros::Rate loop_rate(100);

    int count = 0;

    // Main loop
    while(ros::ok()) {
        // Do stuff
        // E.g. some calculations, create messages, read messages etc.

        // Read messages from relavent interfaces
        auto grabber_motors_msg = grabber_interface.get_msg();
        auto drop_motors_msg = drop_interface.get_msg();
        auto cmd_vel_msg = navigation_interface.get_msg();

        // Create a message and set the current message t be sent
        // This will continue to be sent until a new message is set
        // Get from I2C(?)
        std_msgs::Empty drop_status_msg;
        std_msgs::Empty grabber_status_msg;
        std_msgs::Empty odometry_msg;
        drop_interface.set_msg(drop_status_msg);
        grabber_interface.set_msg(grabber_status_msg);
        navigation_interface.set_msg(odometry_msg);

        // Keep update frequency
        loop_rate.sleep();
    }
}
