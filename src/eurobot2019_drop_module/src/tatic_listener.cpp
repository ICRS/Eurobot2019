#include "ros/ros.h"
#include "std_msgs/String.h"
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{       
        ROS_INFO("Drop node received [%s]", msg->data.c_str());
}       

int main(int argc, char **argv)
{
        ros::init(argc, argv, "tatic_listener");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("tatic",1000, chatterCallback);
        ros::spin();
        return 0
} 
