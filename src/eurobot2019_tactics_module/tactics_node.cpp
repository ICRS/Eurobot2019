/////////////////////////////////////////////////////////////////////////////////////////////////////////
//THIS CODE BLOCK IS OUT OF PLACE, SHOULD BE USED TO FIND A GOAL, GIVEN THE POSITION OF A PUCK
# define APPROACH_RADIUS 3 //distance away from puck robot must be at, to pick it up

// use make_plan to check that place is free
// assuming make_plan returns an empty vector, if occupied space

geometry_msgs::PoseStamped Start;
geometry_msgs::PoseStamped Goal;


Start.header.seq = 0;
Start.header.stamp = ros::Time(0);
Start.header.frame_id = "/map";
Start.pose = pose.pose;

// calculate thing around puck
// set direction/position as x2, y2, cy, sy

double approach_angle = atan2((puck.pose.position.x - pose.pose.position.x), (puck.pose.position.x - pose.pose.position.x));
double x = puck.pose.position.x - (APPROACH_RADIUS * cos(approach_angle));
double y = puck.pose.position.y - (APPROACH_RADIUS * sin(approach_angle));

double desired_yaw = constrainAngle(approach_angle - PI/2);

double cy = cos(desired_yaw * 0.5);
double sy = sin(desired_yaw * 0.5);

Goal.header.seq = 0;
Goal.header.stamp = ros::Time(0);
Goal.header.frame_id = "/map";
Goal.pose.position.x = x;
Goal.pose.position.y = y;
Goal.pose.orientation.z = cy;
Goal.pose.orientation.w = sy;

nav_msgs::GetPlan srv;
srv.request.start = Start;
srv.request.goal = Goal;
srv.request.tolerance = tolerance;

ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));

while(response.plan.poses.size() == 0){
    approach_angle = fmod(rand(), 2*PI);
    Goal.pose.position.x = puck.pose.position.x - (APPROACH_RADIUS * cos(approach_angle));
    Goal.pose.position.y = puck.pose.position.y - (APPROACH_RADIUS * sin(approach_angle));

    desired_yaw = constrainAngle(approach_angle - PI/2);

    cy = cos(desired_yaw * 0.5);
    sy = sin(desired_yaw * 0.5);

    Goal.pose.orientation.z = cy;
    Goal.pose.orientation.w = sy;

    srv.request.goal = Goal;
    check_path.call(srv);
}

ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

goal.target_pose.header.frame_id = "/map";
goal.target_pose.header.stamp = ros::Time::now(0);

goal.target_pose.pose = Goal.pose;

ROS_INFO("Sending goal");
ac.sendGoal(goal);
// CODE BLOCK ENDS HERE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Hardcoded VERSION
#include <ros/ros.h>
#include <eurobot2019_messages/grabber_motors.h>
#include <eurobot2019_messages/pickup.h>
#include <eurobot2019_messages::collision_avoidance.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/MapMetaData.h.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetPlan.h"
#include "std_msgs/Int32.h"
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include "message_interface.hpp"

#define CLOSE_ENOUGH //When goal is close enough, such that we can ignore collision_avoidance around adjacent directions adjacent to goal direction
#define PI 3.1415926
#define K //angle scoring constant
#define Q //adjacency scoring constant
#define COLLISION_AVOIDANCE_MOVE 0.05 //Amount that robot moves away to,to avoid collision
#define TOO_LONG 5000
#define APPROACH_RADIUS 3 //distance away from puck robot must be at, to pick it up

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

eurobot2019_messages::collision_avoidance collision_avoidance;

void sub_collision_avoidance_Callback(const eurobot2019_messages::collision_avoidance::ConstPtr& msg)
{
    ROS_INFO("sub_collision_avoidance_Callback returned successfully");
    collision_avoidance = *msg;
}

geometry_msgs::Pose get_robot_pos(tf::TransformListener& listener, double& yaw);
void wallignore(double yaw, std::vector<int>& wall_vector, geometry_msg::Pose pose);
void wallignore_instance(double yaw, double x, double y, std::vector<char>& wall_vector);
double anglescores(double yaw, double x, double y, int direction);
double constrainAngle(double x);
Quaterniond toQuaternion(double yaw, double pitch, double roll);
bool collision_check(double yaw, geometry_msg::Pose pose, MoveBaseGoal goal, MoveBaseClient ac);
double space_distance(double  yaw, geometry_msgs::Pose pose, int direction);

int main(int argc, char **argv) {
    srand (time(NULL));
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
    ros::Subscriber sub_collision_avoidance = n.subscribe("collision_avoidance", 1, sub_collision_avoidance_Callback);

    ServiceClient check_path = nh_.serviceClient<nav_msgs::getplan>("/move_base/make_plan");

    do{
        nh.spinOnce();
    }while(!got_map && !got_map_metadata);

    double resolution = map_metadata.resolution;
    int width = map_metadata.width;
    int height = map_metadata.height;
    geometry_msg::Pose origin = map_metadeta.origin;

    double collision_radius;
    if(!nh.getParam("tactics/collision_radius",
                     collision_radius)) {
        ROS_ERROR("Failed to get param 'tactics/collision_radius'");
    }

    double tolerance;
    if(!nh.getParam("tactics/tolerance",
                     tolerance)) {
        ROS_ERROR("Failed to get param 'tactics/tolerance'");
    }

    double yaw;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    static auto total_time = std::chrono::high_resolution_clock::now();
    int pucks_taken = 0;
    int current_puck_colour;
    bool is_vertical;

    bool heading_to_ramp = false;

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //Assume first goal is known, ROSPARAM
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.y = -1.0;

    ros::Rate sleeper(100);

    while (ros::ok()){
        //Set next target, has been set?

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        double total_collision_time = 0;

        static auto t1 = std::chrono::high_resolution_clock::now();
        static auto t3 = std::chrono::high_resolution_clock::now();
        bool has_moved_closer_or_in_time = true;
        geometry_msgs::Pose pose = get_robot_pos(listener, yaw);
        double prev_distance = pow(pose.pose.position.x - goal.target_pose.pose.position.x, 2) + pow(pose.pose.position.y - goal.target_pose.pose.position.y, 2);

        while((ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) && (has_moved_closer_or_in_time || heading_to_ramp)){
            bool avoided_collision = false;
            pose = get_robot_pos(listener, yaw);
            double distance = pow(pose.pose.position.x - goal.target_pose.pose.position.x, 2) + pow(pose.pose.position.y - goal.target_pose.pose.position.y, 2);
            nh.spinOnce();
            //check collision avoidance

            static auto t4 = std::chrono::high_resolution_clock::now();
            while(collision_check(yaw, pose, goal.target_pose, ac)){
                avoided_collision = true;
                nh.spinOnce();
                pose = get_robot_pos(listener, yaw);
            }

            if(avoided_collision){
                ac.sendGoal(goal);
                static auto t5 = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> collision_time = t5 - t4;
                total_collision_time += collision_time.count();
            }

            else if(distance < prev_distance){
                prev_distance = distance;
                t1 = std::chrono::high_resolution_clock::now();

                if (too_long_span.count() > TOO_LONG){
                    has_moved_closer_or_in_time = false;
                }
            }

            else{
                auto t2 = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> time_span = t2 - t1;
                std::chrono::duration<double, std::milli> too_long_span = t2 - t3;

                if (time_span.count() > 1000 || too_long_span.count() > TOO_LONG){
                    has_moved_closer_or_in_time = false;
                }
            }
        }

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && !heading_to_ramp){
            ROS_INFO("Hooray, acquired desired position and orientation");
            eurobot2019_messages::pickup pickup;
            pickup.colour = current_puck_colour;
            pickup.pos_reached = true;
            pickup.is_vertical = is_vertical;
            pickup_interface.set_msg(pickup);
            int pickup_status_msg;
            nh.spinOnce();
            pickup_interface.get_msg(pickup_status_msg);

            while(pickup_status_msg > 0){
                nh.spinOnce();
                pickup_interface.get_msg(pickup_status_msg)
                if(is_vertical){
                  if(pickup_status == 4){
                    //back up
                  }
                }
            }

            pucks_taken++;
            // Send idle command
            // Set next goal + determine if should go to ramp
            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> total_time_span = total_time - t1;
            if ((pucks_taken < 8 && total_time_span.count()) > 90000) || pucks_taken >= 8){
               //Set goal to ramp
              pose = get_robot_pos(listener, yaw);
              double desired_yaw = ;//0 or pi?
              double x = ;//x of unloading POSITION
              double y = ;//y of unloading position
              double cy = cos(desired_yaw * 0.5);
              double sy = sin(desired_yaw * 0.5);

              goal.pose.position.x = x;
              goal.pose.position.y = y;
              goal.pose.orientation.z = cy;
              goal.pose.orientation.w = sy;

              heading_to_ramp = true;
            }

            else if(heading_to_ramp){
              //pick up another puck
              heading_to_ramp = false;
            }

            else{
                //Set goal next puck
                std::vector <double> puck_score;
                for(int i = 0; i < poses.poses.size(); i++){
                double puck_distance = pow(poses.poses[i].position.x - pose.pose.position.x, 2) + pow(poses.poses[i].position.y - pose.pose.position.y, 2);

                int puck_value;

                if (puck_colours[i] = 2){
                  puck_value = 8//value for green;
                }
                else if ((puck_colours[i] = 3){
                  puck_value = 12//value for blue;
                }
                else{
                  puck_value = 0;
                }
                puck_score.push_back(puck_value/puck_distance);//Scale by a constant?
              }

              int chosen_puck = std::max_element(puck_score.begin(),puck_score.end()) - puck_score.begin();
              geometry_msgs::Pose puck = poses.poses[chosen_puck];
              current_puck_colour = puck_colours[chosen_puck];
              is_vertical = is_vertical[chosen_puck];

              geometry_msgs::PoseStamped Start;
              geometry_msgs::PoseStamped Goal;

              Start.header.seq = 0;
              Start.header.stamp = ros::Time::now();
              Start.header.frame_id = "/map";
              Start.pose = pose.pose;

              double approach_angle = atan2((puck.pose.position.y - pose.pose.position.y), (puck.pose.position.x - pose.pose.position.x));
              double x = puck.pose.position.x - (APPROACH_RADIUS * cos(approach_angle));
              double y = puck.pose.position.y - (APPROACH_RADIUS * sin(approach_angle));

              double desired_yaw = constrainAngle(approach_angle - PI/2);

              double cy = cos(desired_yaw * 0.5);
              double sy = sin(desired_yaw * 0.5);

              Goal.header.seq = 0;
              Goal.header.stamp = ros::Time::now();
              Goal.header.frame_id = "/map";
              Goal.pose.position.x = x;
              Goal.pose.position.y = y;
              Goal.pose.orientation.z = cy;
              Goal.pose.orientation.w = sy;

              nav_msgs::GetPlan srv;
              srv.request.start = Start;
              srv.request.goal = Goal;
              srv.request.tolerance = tolerance;

              ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));

              while(response.plan.poses.size() == 0){
                  approach_angle = fmod(rand(), 2*PI);
                  Goal.pose.position.x = puck.pose.position.x - (APPROACH_RADIUS * cos(approach_angle));
                  Goal.pose.position.y = puck.pose.position.y - (APPROACH_RADIUS * sin(approach_angle));

                  desired_yaw = constrainAngle(approach_angle - PI/2);

                  cy = cos(desired_yaw * 0.5);
                  sy = sin(desired_yaw * 0.5);

                  Goal.pose.orientation.z = cy;
                  Goal.pose.orientation.w = sy;

                  srv.request.goal = Goal;
                  check_path.call(srv);
              }

              ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

              goal.target_pose.pose = Goal.pose;
            }

            nh.spinOnce();

            sleeper.sleep();
            }
        }

        else{
            ROS_INFO("The base failed to move to next position and orientation");
            //Set goal another puck
            //Assume receiving geometry_msgs::PoseArray poses, array of ints containing colours int* puck_colours[], array of bools containing whether or not is_vertical called is_vertical[]
            std::vector <double> puck_score;
            for(int i = 0; i < poses.poses.size(); i++){
            double puck_distance = pow(poses.poses[i].position.x - pose.pose.position.x, 2) + pow(poses.poses[i].position.y - pose.pose.position.y, 2);

            double proximity_to_prev_goal = pow(poses.poses[i].position.x - goal.target_pose.pose.position.x, 2) + pow(poses.poses[i].position.y - goal.target_pose.pose.position.y, 2);

            int puck_value;

            if (puck_colours[i] = 2){
              puck_value = //value for green;
            }
            else if ((puck_colours[i] = 3){
              puck_value = //value for blue;
            }
            else{
              puck_value = 0;
            }
            puck_score.push_back((puck_value  * pow(proximity_to_prev_goal, (total_collision_time/1000)))/puck_distance);//Scale by a constant?
          }

          int chosen_puck = std::max_element(puck_score.begin(),puck_score.end()) - puck_score.begin();
          geometry_msgs::Pose puck = poses.poses[chosen_puck];
          current_puck_colour = puck_colours[chosen_puck];
          is_vertical = is_vertical[chosen_puck];

          geometry_msgs::PoseStamped Start;
          geometry_msgs::PoseStamped Goal;

          Start.header.seq = 0;
          Start.header.stamp = ros::Time::now();
          Start.header.frame_id = "/map";
          Start.pose = pose.pose;

          if(is_vertical){
            
          }
          double approach_angle = atan2((puck.pose.position.y - pose.pose.position.y), (puck.pose.position.x - pose.pose.position.x));
          double x = puck.pose.position.x - (APPROACH_RADIUS * cos(approach_angle));
          double y = puck.pose.position.y - (APPROACH_RADIUS * sin(approach_angle));

          double desired_yaw = constrainAngle(approach_angle - PI/2);

          double cy = cos(desired_yaw * 0.5);
          double sy = sin(desired_yaw * 0.5);

          Goal.header.seq = 0;
          Goal.header.stamp = ros::Time::now();
          Goal.header.frame_id = "/map";
          Goal.pose.position.x = x;
          Goal.pose.position.y = y;
          Goal.pose.orientation.z = cy;
          Goal.pose.orientation.w = sy;

          nav_msgs::GetPlan srv;
          srv.request.start = Start;
          srv.request.goal = Goal;
          srv.request.tolerance = tolerance;

          ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));

          while(response.plan.poses.size() == 0){
              approach_angle = fmod(rand(), 2*PI);
              Goal.pose.position.x = puck.pose.position.x - (APPROACH_RADIUS * cos(approach_angle));
              Goal.pose.position.y = puck.pose.position.y - (APPROACH_RADIUS * sin(approach_angle));

              desired_yaw = constrainAngle(approach_angle - PI/2);

              cy = cos(desired_yaw * 0.5);
              sy = sin(desired_yaw * 0.5);

              Goal.pose.orientation.z = cy;
              Goal.pose.orientation.w = sy;

              srv.request.goal = Goal;
              check_path.call(srv);
          }

          ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

          goal.target_pose.pose = Goal.pose;
        }

        nh.spinOnce();

        sleeper.sleep();
    }
}

//A function to normalize the angle to -pi to pi
double constrainAngle(double x){
    x = fmod(x + 3.1415926, 6.2831852);
    if (x < 0)
        x += 6.2831852;
    return x - 3.1415926;
}

//A function that assigns scores to different ultrasound directions according to the angle between a ultrasound direction and the direction to the previous(current?) goal
double anglescores(double yaw, double x, double y, int direction){
  double gamma = atan2(y, x);
  // Calculate angle between direction of robot and the goal
  double angler2g = yaw - gamma + 1.57079632;//using 8 decimal places, if bug return to 7

  return constrainAngle(angler2g - 0.78539816*direction);
}

void wallignore(double yaw, std::vector<int>& wall_vector, geometry_msg::Pose pose){
    // Might need to reconsider if pucks are taken to be occupied space
        //relative to origin (edge of map), not (0,0) of map
    double resolution = map_metadata.resolution;
    int width = map_metadata.width;
    int height = map_metadata.height;
    geometry_msg::Pose origin = map_metadeta.origin;

    double base_link_x = pose.pose.position.x - origin.position.x;
    double base_link_y = pose.pose.position.y - origin.position.y;
    int base_link_pixel_x = round((pose.pose.position.x - origin.position.x)/resolution);
    int base_link_pixel_y = round((pose.pose.position.y - origin.position.y)/resolution);

    std::vector<bool> bool_wall_vector = {0, 0, 0, 0, 0, 0, 0, 0};

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
                double y = (j - base_link_pixel_x) * resolution;
                double x = (i - base_link_pixel_y) * resolution;
                std::vector<int> wall_vector_instance;
                wallignore_instance(yaw, x, y, wall_vector_instance);
                for(int i = 0; i < wall_vector_instance.size(); i++){
                    bool_wall_vector[wall_vector_instance[i]] = true;
                }
            }
        }
    }

    for(int i = 0; i < 8; i++){
        if(bool_vector[i]){
            wall_vector.push_back(i);
        }
    }
}

//A function that suppresses the collision avoidance on the wall, x,y are the occupied cell's relative position to the centre of the robot.
void wallignore_instance(double yaw, double x, double y, std::vector<int>& wall_vector_instance){
  //Calculate the anticlockwise angle of the point's position vector to positive x-axis.
  double alpha = atan2(y, x);
  //Calcu;ate the angle (clockwise is positive) between yaw and the point's position vector.
  double angler2w = yaw - alpha + 1.5707963;
  if (((-0.3926991 <= angler2w) && (angler2w <= 0.3926991)) || (((-0.3926991 + 3.1415926*2) <= angler2w) && (angler2w <= (0.3926991 + 3.1415926*2)))){
    // Suppress collision_avoidance[0] // Assume 0 is the ultrasound sensor in the front of the robot, and the order increases in clockwise direction
    wall_vector_instance.push_back(0);
  }
  if (((0.3926991 <= angler2w) && (angler2w <= (-5.1050881 + 3.1415926*2))) || (((0.3926991 + 3.1415926*2) <= angler2w) && (angler2w <= (-5.1050881 + 3.1415926*4)))){
    // Suppress collision_avoidance[1]
    wall_vector_instance.push_back(1);
  }
  if (((-4.7123890 <= angler2w) && (angler2w <= -4.3196899)) || (((-5.1050881 + 3.1415926*2) <= angler2w) && (angler2w <= (-4.3196899 + 3.1415926*2))) || (((-5.1050881 + 3.1415926*4) <= angler2w) && (angler2w <= (7.8539817)))){
    // Suppress collision_avoidance[2] // From the calculation, the range of the angler2w is between -3pi/2 to 5pi/2
    wall_vector_instance.push_back(2);
  }
  if (((-4.3196899 <= angler2w) && (angler2w <= -3.5342917)) || (((-4.3196899 + 3.1415926*2) <= angler2w) && (angler2w <= (-3.5342917 + 3.1415926*2)))){
    // Suppress collision_avoidance[3]
    wall_vector_instance.push_back(3);
  }
  if (((-3.5342917 <= angler2w) && (angler2w <= -2.7488936)) || (((-3.5342917 + 3.1415926*2) <= angler2w) && (angler2w <= (-2.7488936 + 3.1415926*2)))){
    // Suppress collision_avoidance[4]
    wall_vector_instance.push_back(4);
  }
  if (((-2.7488936 <= angler2w) && (angler2w <= -1.9634954)) || (((-2.7488936 + 3.1415926*2) <= angler2w) && (angler2w <= (-1.9634954 + 3.1415926*2)))){
    // Suppress collision_avoidance[5]
    wall_vector_instance.push_back(5);
  }
  if (((-1.9634954 <= angler2w) && (angler2w <= -1.1780972)) || (((-1.9634954 + 3.1415926*2) <= angler2w) && (angler2w <= (-1.1780972 + 3.1415926*2)))){
    // Suppress collision_avoidance[6]
    wall_vector_instance.push_back(6);
  }
  if (((-1.1780972 <= angler2w) && (angler2w <= -0.3926991)) || (((-1.1780972 + 3.1415926*2) <= angler2w) && (angler2w <= (-0.3926991 + 3.1415926*2)))){
    // Suppress collision_avoidance[7]
    wall_vector_instance.push_back(7);
  }
}

Quaterniond toQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double cr = cos(roll * 0.5);

    Quaterniond q;
    q.w = cy * cp * cr +
    q.x =  -
    q.y =  +
    q.z = sy * cp * cr - ;
    return q;
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

double space_distance(double  yaw, geometry_msgs::Pose pose, int direction){

    double base_link_x = pose.pose.position.x - map.origin.position.x;
    double base_link_y = pose.pose.position.y - map.origin.position.y;
    double motion_angle = yaw - (direction * PI/4) + PI/2;
    int base_link_pixel_x = round((base_link_x)/resolution);
    int base_link_pixel_y = round((base_link_y)/resolution);
    bool found_occupied = false;

      for(int i = base_link_pixel_x; i >= 0 && i <= width && found_occupied; i++){
          base_link_x += resolution
          base_link_y += resoultion * tan(motion_angle)

          int pixel_x = round((base_link_x)/resolution);
          int pixel_y = round((base_link_y)/resolution);

          if(pixel_x <= 0 || pixel_y <= 0 || pixel_x >= width || pixel_y >= height){
              found_occupied = true;
              double distance_to_occupied = pow((base_link_x - (pose.pose.position.x - map.origin.position.x)), 2) + pow((base_link_y - (pose.pose.position.y - map.origin.position.y)), 2)
          }

          else{
              if(map.data[pixel_x + pixel_y * width] == 100){
                  found_occupied = true;
                  double distance_to_occupied = pow((base_link_x - (pose.pose.position.x - map.origin.position.x)), 2) + pow((base_link_y - (pose.pose.position.y - map.origin.position.y)), 2)
              }
          }
      }
}

bool collision_check(double yaw, geometry_msg::Pose pose, move_base_msgs::MoveBaseGoal goal, MoveBaseClient ac){

bool blocked_by_non_wall = false;
std::vector<int> blocked_directions;
blocked_directions.push_back(15); //junk value to push

std::vector<int> wall_vector;
wallignore(yaw, wall_vector, pose);

for(int i = 0, j = 0; i < 8; i++){
  if (i == wall_vector[j]){
    blocked_directions.push_back(i);

    if (j < wall_vector.size()){
      j++;
    }
  }

  else{
    if(collision_avoidance[i]){
      blocked_by_non_wall = true;
      blocked_directions.push_back(i);
    }
  }
}

if(blocked_by_non_wall){
  ac.cancelGoal();

  //ignore directions between two blocked_directions, unless super close
  std::vector<double> scores;
  std::vector<int> directions;

  move_base_msgs::MoveBaseGoal new_goal;

  new_goal.target_pose.header.frame_id = "/base_link";
  new_goal.target_pose.header.stamp = ros::Time::now();

  for(int i = 0, j = 0, r = 0; i < 8; i++){
    if(i == blocked_directions[j + 1]){
      if (j < blocked_directions.size() - 1){
        j++;
        r++;
      }
    }

    else{
      if(i == 0){
        if(blocked_directions[1] == 1 && blocked_directions[blocked_directions.size() - 1] == 7){
            if (pow(goal.target_pose.pose.position.x - pose.pose.position.x, 2) + pow(goal.target_pose.pose.position.y - pose.pose.position.y, 2)) <= CLOSE_ENOUGH){
                std::vector<int> wall_vector_instance;
                wallignore_instance(yaw, goal.target_pose.pose.position.x - pose.pose.position.x, goal.target_pose.pose.position.y - pose.pose.position.y, wall_vector_instance);
                for(int k = 0; k < wall_vector_instance.size(); k++){
                    if(wall_vector_instance[k] == i){
                        ROS_INFO("Sending goal");
                        ac.sendGoal(goal);
                        return false;
                    }
                }
            }
        }


        else if(blocked_directions[1] == 1 || blocked_directions[blocked_directions.size() - 1] == 7){
            double distance_to_goal = space_distance(yaw, pose, i);
            double angle_to_goal = fabs(anglescores(yaw, goal.target_pose.pose.position.x - pose.pose.position.x, goal.target_pose.pose.position.y - pose.pose.position.y, i));
            scores.push_back((distance_to_occupied_squared*K)/(angle_to_goal*Q));
            directions.push_back(i);
          //adjust score accordingly
        }

        else{
            double distance_to_goal = space_distance(yaw, pose, i);
            double angle_to_goal = fabs(anglescores(yaw, goal.target_pose.pose.position.x - pose.pose.position.x, goal.target_pose.pose.position.y - pose.pose.position.y, i);
            scores.push_back((distance_to_occupied_squared*K)/angle_to_goal);
            directions.push_back(i);
          //none of them are, no score penalty
          //find rest of score, i.e. space(distance to closest occupied point in direction), angle from goal
        }
      }

      if(i == 7){
        if(blocked_directions[1] == 0 && blocked_directions[blocked_directions.size() - 1] == 6){
            if (pow(goal.target_pose.pose.position.x - pose.pose.position.x, 2) + pow(goal.target_pose.pose.position.y - pose.pose.position.y, 2)) <= CLOSE_ENOUGH){
                std::vector<int> wall_vector_instance;
                wallignore_instance(yaw, goal.target_pose.pose.position.x - pose.pose.position.x, goal.target_pose.pose.position.y - pose.pose.position.y, wall_vector_instance);
                for(int k = 0; k < wall_vector_instance.size(); k++){
                    if(wall_vector_instance[k] == i){
                        ROS_INFO("Sending goal");
                        ac.sendGoal(goal);
                        return false;
                    }
                }
            }
        }


        else if(blocked_directions[1] == 0 || blocked_directions[blocked_directions.size() - 1] == 6){
            double distance_to_goal = space_distance(yaw, pose, i);
            double angle_to_goal = fabs(anglescores(yaw, goal.target_pose.pose.position.x - pose.pose.position.x, goal.target_pose.pose.position.y - pose.pose.position.y, i));
            scores.push_back((distance_to_occupied_squared*K)/(angle_to_goal*Q));
            directions.push_back(i);
          //adjust score accordingly
        }

        else{
            double distance_to_goal = space_distance(yaw, pose, i);
            double angle_to_goal = fabs(anglescores(yaw, goal.target_pose.pose.position.x - pose.pose.position.x, goal.target_pose.pose.position.y - pose.pose.position.y, i));
            scores.push_back((distance_to_occupied_squared*K)/angle_to_goal);
            directions.push_back(i);
          //none of them are, no score penalty
          //find rest of score, i.e. space(distance to closest occupied point in direction), angle from goal
        }
      }

      else{
        if(blocked_directions[r] == i - 1 && blocked_directions[r + 1] == i + 1){
            if (pow(goal.target_pose.pose.position.x - pose.pose.position.x, 2) + pow(goal.target_pose.pose.position.y - pose.pose.position.y, 2)) <= CLOSE_ENOUGH){
                std::vector<int> wall_vector_instance;
                wallignore_instance(yaw, goal.target_pose.pose.position.x - pose.pose.position.x, goal.target_pose.pose.position.y - pose.pose.position.y, wall_vector_instance);
                for(int k = 0; k < wall_vector_instance.size(); k++){
                    if(wall_vector_instance[k] == i){
                        ROS_INFO("Sending goal");
                        ac.sendGoal(goal);
                        return false;
                    }
                }
            }
        }

        else if(blocked_directions[r] == i - 1 || blocked_directions[r + 1] == i + 1){
            double distance_to_goal = space_distance(yaw, pose, i);
            double angle_to_goal = fabs(anglescores(yaw, goal.target_pose.pose.position.x - pose.pose.position.x, goal.target_pose.pose.position.y - pose.pose.position.y, i));
            scores.push_back((distance_to_occupied_squared*K)/(angle_to_goal*Q));
            directions.push_back(i);
          //adjust score accordingly
          // Calculate rest of score
        }

        else{
            double distance_to_occupied_squared = space_distance(yaw, pose, i);
            double angle_to_goal = fabs(anglescores(yaw, goal.target_pose.pose.position.x - pose.pose.position.x, goal.target_pose.pose.position.y - pose.pose.position.y, i));

            scores.push_back((distance_to_occupied_squared*K)/angle_to_goal);
            directions.push_back(i);
            //none of them are, no score penalty
            //find rest of score, i.e. space(distance to closest occupied point in direction), angle from goal
          }
        }
      }
    }

    if(scores.size() == 0){
        return true;
    }

    double max_score = 0;
    int max_index;

    for(int i = 0; i < scores.size(); i++){
        if(max_score < scores[i]){
            max_score = score[i];
            max_index = i;
        }
    }
    //assuming +ve x is forward and +ve y is left
    new_goal.target_pose.pose.position.x = COLLISION_AVOIDANCE_MOVE * sin(PI/2 - ((directions[max_index]*PI)/4));
    new_goal.target_pose.pose.position.y = -(COLLISION_AVOIDANCE_MOVE * cos(PI/2 - ((directions[max_index]*PI)/4)));

    ROS_INFO("Sending goal");
    ac.sendGoal(new_goal);

    return true;
  }
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
