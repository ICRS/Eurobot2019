//Hardcoded VERSION
#include <ros/ros.h>
#include <eurobot2019_messages/grabber_motors.h>
#include <eurobot2019_messages/pickup.h>
#include <queue>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "message_interface.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Function to check if floating point numbers are close enough
bool approx_equal(float a, float b) {
    return fabs(a-b) < 1e-2;
}

int main(int argc, char **argv) {
    // Initialise ROS
    ros::init(argc, argv, "tactics");

    // Get node handle
    ros::NodeHandle nh;

    // Create interface to the pickup node
    MessageInterface<eurobot2019_messages::pickup, std_msgs::Int32>
                command(10, "pickup_status", 10, "pickup");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //Declare variable PoseStamped, from launch file (includes at least first 2 desired poses)

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    //goal.target_pose.pose.position.x = 1.0;
    //goal.target_pose.pose.orientation.w = 1.0;
    //Set next target

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, acquired desired position and oreintation");
    else
        ROS_INFO("The base failed to move to next position and oreintation");

    // 100 Hz update rate
    ros::Rate sleeper(100);

    while(ros::ok()) {


        /*Planned:
Tactics

Start in a predefined place
* Set a goal to move in predefined path
	Timeout for whether or not robot is getting closer to goal
		Go to new goal
	Listen to colision avoidance (unless next to wall)
		Move in opposite direction
		Set new goal?
	Wait for SUCCEEDED

Get in position for pick up
Pickup

Set next goal
repeat

After some point (time, position, pucks taken, e.t.c., e.g. If not 8 pucks by 90 seconds, head to ramp)
	Set goal to ramp
	Drop things*/

        // Maintain 100 Hz
        sleeper.sleep();
    }
}
