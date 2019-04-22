//Hardcoded VERSION
#include <ros/ros.h>
#include <eurobot2019_messages/grabber_motors.h>
#include <eurobot2019_messages/pickup.h>
#include <queue>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <move_base_msgs/MoveBaseAction.h>
#inclde "geometry_msgs/PoseStamped"
#include <actionlib/client/simple_action_client.h>

#include "message_interface.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Quaterniond{
    double w;
    double x;
    double y;
    double z;
};

// Function to check if floating point numbers are close enough
bool approx_equal(float a, float b) {
    return fabs(a-b) < 1e-2;
}

int main(int argc, char **argv) {
    // Initialise ROS
    ros::init(argc, argv, "tactics");

    // Get node handle
    ros::NodeHandle nh;

    //To get position
    //Transform from base_link to map
    tf::TransformListener listener;
    tf::TransformStamped transform;

/* function that returns current point
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(/map, /base_link, ros::Time(0), transform);

      // construct a pose message
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = map_frame;
      pose_stamped.header.stamp = ros::Time::now();

      pose_stamped.pose.orientation.x = transform.getRotation().getX();
      pose_stamped.pose.orientation.y = transform.getRotation().getY();
      pose_stamped.pose.orientation.z = transform.getRotation().getZ();
      pose_stamped.pose.orientation.w = transform.getRotation().getW();

      pose_stamped.pose.position.x = transform.getOrigin().getX();
      pose_stamped.pose.position.y = transform.getOrigin().getY();
      pose_stamped.pose.position.z = transform.getOrigin().getZ();

      if(is_stamped)
        p_pub.publish(pose_stamped);
      else
        p_pub.publish(pose_stamped.pose);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
    }

} */


    listener.lookupTransform("/base_link", "/map",
                               ros::Time(0), transform);

    // Create interface to the pickup node
    MessageInterface<eurobot2019_messages::pickup, std_msgs::Int32>
                pickup_interface(10, "pickup", 10, "pickup_status");

    MessageInterface<eurobot2019_messages::drop_command, std_msgs::Int32>
                drop_interface(10, "drop", 10, "drop_status_l");

    MessageSubscriberInterface<std_msgs::Int32>
                drop_status_right(10, "drop_status_r");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    static auto total_time = std::chrono::high_resolution_clock::now();
    int pucks_taken = 0;

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter to the right
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.y = -1.0;

    while (ros::ok()){
    //Set next target, has been set?

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    static auto t1 = std::chrono::high_resolution_clock::now();
    bool has_moved_closer = true;

    while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || has_moved_closer) {//check collision avoidance also
        if(distance < prev_distance){
            prev_distance = distance;
            static auto t1 = std::chrono::high_resolution_clock::now();
        }

        else {
        auto t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> time_span = t2 - t1;

            if (time_span.count() > 1000){
                has_moved_closer = false;
            }
        }
    }

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, acquired desired position and orientation");
        //Send pickup command

        while(pickup_interface.get_msg(motor_msg) > 0){

        }

        pucks_taken++;
        // Send idle command
        // Set next goal + determine if should go to ramp
        auto t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> time_span = total_time - t1;
        if ((pucks_taken < 8 && time_span(count) > 90000) || pucks_taken >= 8){
            //Set goal to ramp
        }

        else{
            //Set goal next puck
        }
    }

    else{
        ROS_INFO("The base failed to move to next position and orientation");
        //Set new goal
    }
}



/*
    // 100 Hz update rate
    ros::Rate sleeper(100);

    while(ros::ok()) {
        // Maintain 100 Hz
        sleeper.sleep();
    }

*/
}

Quaterniond toQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaterniond q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}



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
