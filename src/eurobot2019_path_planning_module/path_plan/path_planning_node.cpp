#include <ros/ros.h>

#include "tactics_interface.hpp"

int main(int argc, char **argv) {
    // Init ROS
    ros::init(argc, argv, "path_plan_module");
    
    // Get node handle
    ros::NodeHandle node_handle;

    // Create TacticsInterface
    TacticsInterface tactics_interface(&node_handle, 100, 10);

    // Control loop rate to be 100 Hz
    ros::Rate loop_rate(100);

    // Main loop
    while(ros::ok()) {
        // Do stuff
        
        std_msgs::Empty msg;
        tactics_interface.publish(msg);

        // Check for updates
        ros::spinOnce();

        // Keep update frequency
        loop_rate.sleep();
    }
}
