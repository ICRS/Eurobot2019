#ifndef __TACTICS_INTERFACE_HPP__
#define __TACTICS_INTERFACE_HPP__

#include <ros/ros.h>
#include <std_msgs/Empty.h>

class TacticsInterface {
public:
    TacticsInterface(ros::NodeHandle *nh, 
                     float frequency_pub,
                     float frequency_sub);
    ~TacticsInterface();
    
    // Publish a message to the tactics node
    void publish(std_msgs::Empty msg);
private:
   // The publisher in charge of publishing to our topic
   ros::Publisher m_publisher;
   // The subscriber in charge of receiving commands from tactics
   ros::Subscriber m_subscriber;

   // Callback function for when a tactics message is received
   void receive_msg(const std_msgs::Empty::ConstPtr& msg);
};

#endif // __TACTICS_INTERFACE_HPP__
