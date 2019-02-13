#include "tactics_interface.hpp"

TacticsInterface::TacticsInterface(ros::NodeHandle *nh, 
                                   float frequency_pub,
                                   float frequency_sub) {
    m_publisher = nh->advertise<std_msgs::Empty>("path_planning_result", 
                                                 frequency_pub);
    m_subscriber = nh->subscribe("path_planning_cmd",
                                 frequency_sub,
                                 &TacticsInterface::receive_msg,
                                 this);
}

TacticsInterface::~TacticsInterface() {
    // dtor
}

void TacticsInterface::publish(std_msgs::Empty msg) {
    m_publisher.publish(msg);
}

void TacticsInterface::receive_msg(const std_msgs::Empty::ConstPtr& msg) {
    // Do sth
}
