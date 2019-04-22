/////////////////////////////////////////////////////////////////////////////////////////////////////////
//THIS CODE BLOCK IS OUT OF PLACE, SHOULD BE USED TO FOR COLLISION_AVOIDANCE, TO IGNORE WALLS
// Might need to reconsider if pucks are taken to be occupied space
    //relative to origin (edge of map), not (0,0) of map
    double base_link_x = pose.pose.position.x - origin.position.x;
    double base_link_y = pose.pose.position.y - origin.position.y;
    int base_link_pixel_x = round((pose.pose.position.x - origin.position.x)/resolution);
    int base_link_pixel_y = round((pose.pose.position.y - origin.position.y)/resolution);

    if ( (base_link_y + collision_radius) > resolution * height){
        int top = height
    }

    else{
        int top = ceil((base_link_y + collision_radius)/resolution)
    }

    if ( (base_link_x + collision_radius) > resolution * width){
        int right = width
    }

    else{
        int right = ceil((base_link_x + collision_radius)/resolution)
    }

    if ( (base_link_x - collision_radius) < 0){
        int left = 0
    }

    else{
        int left = floor((base_link_x - collision_radius)/resolution)
    }

    if ( (base_link_y - collision_radius) < 0){
        int bottom = 0
    }

    else{
        int bottom = ceil((base_link_y - collision_radius)/resolution)
    }

    for(int j = bottom; j <= top; j++){
        for(int i = left; i <= right; i++){
            if(map.data[i + j * width] == 100){
                //double y = (j - base_link_pixel_x) * resolution;
                //double x = (i - base_link_pixel_y) * resolution;
                //yaw
                //run function to ignore things
            }
        }
    }
// CODE BLOCK ENDS HERE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////
//THIS CODE BLOCK IS OUT OF PLACE, SHOULD BE USED TO FIND A GOAL, GIVEN THE POSITION OF A PUCK



// CODE BLOCK ENDS HERE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////









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

std_msgs::Int32 drop_command_r;

void sub_d_r_Callback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("sub_d_r_Callback returned drop_command: [%d, %d, %d]", msg->left, msg->right, msg->middle);
    drop_command_r  = *msg;
}

bool got_map = false;
bool got_map_metadata = false;
nav_msgs::OccupancyGrid map;
nav_msgs::MapMetaData map_metadata;

void sub_map_Callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("sub_map_metadeta_Callback returned map_metadeta");
    map = *msg;
    got_map = true;
}

void sub_map_metadeta_Callback(const nav_msgs::MapMetaData::ConstPtr& msg)
{
    ROS_INFO("sub_map_metadeta_Callback returned a map");
    map_metadata = *msg;
    got_map_metadata = true;
}

geometry_msgs::Pose get_robot_pos(tf::TransformListener& listener, double& yaw){
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

      // construct a pose message
      geometry_msgs::Pose pose;

      pose.pose.orientation.x = transform.getRotation().getX();
      pose.pose.orientation.y = transform.getRotation().getY();
      pose.pose.orientation.z = transform.getRotation().getZ();
      pose.pose.orientation.w = transform.getRotation().getW();

      pose.pose.position.x = transform.getOrigin().getX();
      pose.pose.position.y = transform.getOrigin().getY();
      pose.pose.position.z = transform.getOrigin().getZ();

      yaw = tf::getYaw(transform.getRotation());

      return pose;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
        continue;
    }
}

int main(int argc, char **argv) {
    // Initialise ROS
    ros::init(argc, argv, "tactics");

    // Get node handle
    ros::NodeHandle nh;

    //To get position
    //Transform from base_link to map
    tf::TransformListener listener;

    // Create interface to the pickup node
    MessageInterface<eurobot2019_messages::pickup, std_msgs::Int32>
                pickup_interface(10, "pickup", 10, "pickup_status");

    MessageInterface<eurobot2019_messages::drop_command, std_msgs::Int32>
                drop_interface(10, "drop", 10, "drop_status_l");

    ros::Subscriber sub_d_r = n.subscribe("drop_status_r", 10, sub_d_r_chatterCallback);
    ros::Subscriber sub_map = n.subscribe("map", 1, sub_map_chatterCallback);
    ros::Subscriber sub_map_metadeta = n.subscribe("map", 1, sub_map_metadeta_Callback);

    do{
        nh.spinOnce();
    }while(!got_map && !got_map_metadata);

    double resolution = map_metadata.resolution;
    int width = map_metadata.width;
    int height = map_metadata.height;
    geometry_msg::Pose origin= map_metadeta.origin;

    double collision_radius;
    if(!nh->getParam("tactics/collision_radius",
                     collision_radius)) {
        ROS_ERROR("Failed to get param 'tactics/collision_radius'");
    }

    double yaw;

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
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.y = -1.0; //it's not -1

    ros::Rate sleeper(100);

    while (ros::ok()){
        //Set next target, has been set?

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        static auto t1 = std::chrono::high_resolution_clock::now();
        bool has_moved_closer = true;
        geometry_msgs::Pose pose = get_robot_pos(listener);
        double distance = pow(pose.pose.position.x - goal.target_pose.pose.position.x, 2) + pow(pose.pose.position.y - goal.target_pose.pose.position.y, 2);

        while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || has_moved_closer) {//check collision avoidance also
            tf::TransformStamped pose = get_robot_pos(listener);
            double prev_distance = pow(pose.pose.position.x - goal.target_pose.pose.position.x, 2) + pow(pose.pose.position.y - goal.target_pose.pose.position.y, 2);
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
                //nh.spinOnce();
            }

            pucks_taken++;
            // Send idle command
            // Set next goal + determine if should go to ramp
            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> total_time_span = total_time - t1;
            if ((pucks_taken < 8 && total_time_span.count()) > 90000) || pucks_taken >= 8){
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

        nh.spinOnce();

        sleeper.sleep();
    }
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
