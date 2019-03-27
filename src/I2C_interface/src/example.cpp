#include <ros/ros.h>

#include "message_interface.hpp"
#include <string>
#include <sstream>

int main(int argc, char **argv) {
    // Init ROS
    ros::init(argc, argv, "path_plan_module");

    // Get node handle
    ros::NodeHandle node_handle;

    // Create Tactics interface
    MessageInterface<std_msgs::Empty, std_msgs::Empty>
                tactics_interface(100, "path_planning_result",
                                  10, "path_planning_cmd");

    // Create Localisaion interface
    MessageSubscriberInterface<std_msgs::Empty>
                localisation_interface(50, "localisation_position");

    // Create Path Follower interface
    MessagePublisherInterface<std_msgs::Empty>
                path_follower_interface(20, "path_follower_interface");

    // Control loop rate to be 100 Hz
    ros::Rate loop_rate(100);

    int count = 0;

    // Main loop
    while(ros::ok()) {
        // Do stuff
        // E.g. some calculations, create messages, read messages etc.

        // Read messages from relavent interfaces
        auto received_tactics_msg = tactics_interface.get_msg();
        auto received_localisation_msg = localisation_interface.get_msg();

        // Create a message and set the current message t be sent
        // This will continue to be sent until a new message is set
        std_msgs::Empty path_msg;
        path_follower_interface.set_msg(path_msg);

        // Keep update frequency
        loop_rate.sleep();
    }
}
