////Hardcoded VERSION
#include <ros/ros.h>
#include <eurobot2019_messages/collision_avoidance.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetPlan.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <algorithm>
#include "message_interface.hpp"

#define CLOSE_ENOUGH 1.0//When goal is close enough, such that we can ignore collision_avoidance around adjacent directions adjacent to goal direction
#define PI 3.1415926
#define K 1.0//angle scoring constant
#define Q 1.0//adjacency scoring constant
#define COLLISION_AVOIDANCE_MOVE 0.05 //Amount that robot moves away to,to avoid collision
#define TOO_LONG 5000
#define APPROACH_RADIUS 0.12 //distance away from puck robot must be at, to pick it up

geometry_msgs::PoseArray poses;

// Hardcoded pucks
// sorry guys :'(
int pucks_color[32] = {3,3,1,1,2,1,1,2,2,1,3,1,2,2,3,2,1,1,2,3,1,2,1,3,1,2,1,2,1,3,1,2};
//int poses_array[2][32] = {{0,1300},{0,1700},{450,450},{750,450},{1050,500},{450,2500},{750,2500},{1050,2500},{1050,925},{1050,1075},{975,1000},{1125,1000},{1800,834},{1800,2166},{2000,125},{2000,225},{2000,325},{2000,2675},{2000,2775},{2000,2875},{1543,500},{1543,600},{1543,700},{1543,800},{1543,900},{1543,1000},{1543,2500},{1543,2400},{1543,2300},{1543,2200},{1543,2100},{1543,2000}};
double poses_array[4] = {0.24, 1.80, 1.29, 0.86};
bool isVertical[32] ={1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Function to check if floating point numbers are close enough
bool approx_equal(float a, float b) {
    return fabs(a-b) < 1e-2;
}

std_msgs::Int32 drop_command_r;

void sub_d_r_chatterCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("sub_d_r_chatterCallback returned queue: [%d]", msg->data);
    drop_command_r  = *msg;
}

bool got_map = false;
bool got_map_metadata = false;
nav_msgs::OccupancyGrid map;
nav_msgs::MapMetaData map_metadata;

void sub_map_chatterCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("sub_map_metadata_chatterCallback returned map_metadata");
    map = *msg;
    got_map = true;
}

void sub_map_metadata_chatterCallback(const nav_msgs::MapMetaData::ConstPtr& msg)
{
    ROS_INFO("sub_map_metadata_chatterCallback returned a map");
    map_metadata = *msg;
    got_map_metadata = true;
}

eurobot2019_messages::collision_avoidance collision_avoidance;
bool start = false;

void sub_collision_avoidance_chatterCallback(const eurobot2019_messages::collision_avoidance::ConstPtr& msg)
{
    ROS_INFO("sub_collision_avoidance_chatterCallback returned successfully");
    collision_avoidance = *msg;
    start = true;
}

geometry_msgs::Pose get_robot_pos(tf::TransformListener& listener, double& yaw);
void wallignore(double yaw, std::vector<int>& wall_vector, geometry_msgs::Pose pose, double collision_radius);
void wallignore_instance(double yaw, double x, double y, std::vector<int>& wall_vector);
double anglescores(double yaw, double x, double y, int direction);
double constrainAngle(double x);
bool collision_check(double yaw, geometry_msgs::Pose pose, double collision_radius);
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

    // Create interface to the grabber
    MessageInterface<std_msgs::Int8MultiArray, std_msgs::Int8MultiArray>
                grabber_interface(10, "grabber_command", 10, "grabber_status");

    ros::Subscriber sub_map = nh.subscribe("map", 1, sub_map_chatterCallback);
    ros::Subscriber sub_map_metadata = nh.subscribe("map_metadata", 1, sub_map_metadata_chatterCallback);
    ros::Subscriber sub_collision_avoidance = nh.subscribe("collision_avoidance", 1, sub_collision_avoidance_chatterCallback);

    ros::ServiceClient check_path = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

    do{
        ros::spinOnce();
    }while(!got_map && !got_map_metadata);

    double resolution = map_metadata.resolution;
    int width = map_metadata.width;
    int height = map_metadata.height;
    geometry_msgs::Pose origin = map_metadata.origin;

    double collision_radius;
    if(!nh.getParam("tactics/collision_radius",
                     collision_radius)) {
        ROS_ERROR("Failed to get param 'tactics/collision_radius'");
    }

    signed char flipper;
    signed char[4] grabber_msg_data;
    if(!nh.getParam("tactics/flipper")) {
        ROS_ERROR("Failed to get param 'tactics/flipper'");
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
    int blue_pucks = 0;
    int green_pucks = 0;
    int red_pucks = 0;
    int score = 0;

    int pose_counter = 2;

    geometry_msgs::Point green;
    geometry_msgs::Point red;
    geometry_msgs::Point ramp;
    double ramp_yaw;

    if(!nh.getParam("tactics/green_x",
                     green.x)) {
        ROS_ERROR("Failed to get param 'tactics/green_x'");
    }

    if(!nh.getParam("tactics/green_y",
                     green.y)) {
        ROS_ERROR("Failed to get param 'tactics/green_y'");
    }

    if(!nh.getParam("tactics/red_x",
                     red.x)) {
        ROS_ERROR("Failed to get param 'tactics/red_x'");
    }

    if(!nh.getParam("tactics/red_y",
                     red.y)) {
        ROS_ERROR("Failed to get param 'tactics/red_y'");
    }

    if(!nh.getParam("tactics/ramp_x",
                     ramp.x)) {
        ROS_ERROR("Failed to get param 'tactics/ramp_x'");
    }

    if(!nh.getParam("tactics/ramp_y",
                     ramp.y)) {
        ROS_ERROR("Failed to get param 'tactics/ramp_y'");
    }

    if(!nh.getParam("tactics/ramp_yaw",
                     ramp_yaw)) {
        ROS_ERROR("Failed to get param 'tactics/ramp_y'");
    }


    bool heading_to_ramp = false;
    bool final_stage = false;

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //Assume first goal is known, ROSPARAM
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.y = 0.24;
    goal.target_pose.pose.position.x = 1.80;

    while(!start){
        ros::spinOnce();
    }
    //SECOND ONE, x = 0.86, y = 1.29

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
        double prev_distance = pow(pose.position.x - goal.target_pose.pose.position.x, 2) + pow(pose.position.y - goal.target_pose.pose.position.y, 2);

        while((ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) && (has_moved_closer_or_in_time || heading_to_ramp)){
            bool avoided_collision = false;
            pose = get_robot_pos(listener, yaw);
            double distance = pow(pose.position.x - goal.target_pose.pose.position.x, 2) + pow(pose.position.y - goal.target_pose.pose.position.y, 2);
            ros::spinOnce();
            //check collision avoidance

            static auto t4 = std::chrono::high_resolution_clock::now();
            bool check_collision = collision_check(yaw, pose, collision_radius);
            while(check_collision){
                ac.cancelGoal();
                check_collision = collision_check(yaw, pose, collision_radius);
                avoided_collision = true;
                ros::spinOnce();
                pose = get_robot_pos(listener, yaw);
            }

            if(avoided_collision){
                ac.sendGoal(goal);
                //static auto t5 = std::chrono::high_resolution_clock::now();
                //std::chrono::duration<double, std::milli> collision_time = t5 - t4;
                //total_collision_time += collision_time.count();
            }
            /*
            else if(distance < prev_distance){
                prev_distance = distance;
                t1 = std::chrono::high_resolution_clock::now();
                auto t2 = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> too_long_span = t2 - t3;
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
            }*/
        }

        if(pose_counter == 3){
            //slap puck
        }

        goal.target_pose.pose.position.y = poses_array[2];
        goal.target_pose.pose.position.x = poses_array[3];

        pose_counter += 2;


        ros::spinOnce();

        sleeper.sleep();
    }
}
/*
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && !heading_to_ramp){
            ROS_INFO("Hooray, acquired desired position and orientation");
            eurobot2019_messages::pickup pickup;
            pickup.colour = current_puck_colour;
            if(current_puck_colour == 1){
                red_pucks++;
            }

            if(current_puck_colour == 2){
                green_pucks++;
            }

            else if(current_puck_colour == 3){
                blue_pucks++;
            }
            pickup.pos_reached = true;
            pickup.is_vertical = is_vertical;
            pickup_interface.set_msg(pickup);
            std_msgs::Int32 pickup_status_msg;
            ros::spinOnce();
            pickup_status_msg = pickup_interface.get_msg();

            while(pickup_status_msg.data > 0){
                ros::spinOnce();
                pickup_status_msg = pickup_interface.get_msg();
                if(is_vertical){
                  if(pickup_status_msg.data == 4){
                      pose = get_robot_pos(listener, yaw);
                      double desired_yaw = PI/2;
                      double x = pose.position.x + 0.05;
                      double y = pose.position.y;
                      double cy = cos(desired_yaw * 0.5);
                      double sy = sin(desired_yaw * 0.5);

                      goal.target_pose.pose.position.x = x;
                      goal.target_pose.pose.position.y = y;
                      goal.target_pose.pose.orientation.z = cy;
                      goal.target_pose.pose.orientation.w = sy;

                      ROS_INFO("Sending goal");
                      ac.sendGoal(goal);

                      while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                          bool avoided_collision = false;
                          bool check_collision = collision_check(yaw, pose, goal, ac, collision_radius);

                          while(check_collision){
                              check_collision = collision_check(yaw, pose, goal, ac, collision_radius);
                              avoided_collision = true;
                              ros::spinOnce();
                              pose = get_robot_pos(listener, yaw);
                          }

                          if(avoided_collision){
                              ac.cancelGoal();
                              pose = get_robot_pos(listener, yaw);
                              ros::spinOnce();
                              if(collision_avoidance.data[0]){
                                  double desired_yaw = PI/2;
                                  double x = pose.position.x + 0.05;
                                  double y = pose.position.y;
                                  double cy = cos(desired_yaw * 0.5);
                                  double sy = sin(desired_yaw * 0.5);

                                  goal.target_pose.pose.position.x = x;
                                  goal.target_pose.pose.position.y = y;
                                  goal.target_pose.pose.orientation = pose.orientation;
                                  ac.sendGoal(goal);
                              }
                          }
                      }
                      pickup.has_backed_up = true;
                      pickup_interface.set_msg(pickup);
                  }
                }
            }

            pucks_taken++;
            // Send idle command?
            // Set next goal + determine if should go to ramp
            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> total_time_span = total_time - t1;
            if ((((pucks_taken < 4 && total_time_span.count()) > 80000) || pucks_taken >= 4) && !final_stage){//rethink this
               //Set goal to ramp
              pose = get_robot_pos(listener, yaw);
              ros::spinOnce();
              double desired_yaw = ramp_yaw;//0 or pi?
              double x = ramp.x;//x of unloading POSITION
              double y = ramp.y;//y of unloading position
              double cy = cos(desired_yaw * 0.5);
              double sy = sin(desired_yaw * 0.5);

              goal.target_pose.pose.position.x = x;
              goal.target_pose.pose.position.y = y;
              goal.target_pose.pose.orientation.z = cy;
              goal.target_pose.pose.orientation.w = sy;

              heading_to_ramp = true;
            }

            else if(final_stage){
                auto t2 = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> total_time_span = total_time - t1;
                if ((((pucks_taken < 4 && total_time_span.count()) > 80000) || pucks_taken >= 3)){
                    //head to box, based on colours in container
                    if(green_pucks > 0){
                        geometry_msgs::PoseStamped Start;
                        geometry_msgs::PoseStamped Goal;

                        Start.header.seq = 0;
                        Start.header.stamp = ros::Time::now();
                        Start.header.frame_id = "/map";
                        Start.pose = pose;

                        double approach_angle = atan2((green.y - pose.position.y), (green.x - pose.position.x));
                        double x = green.x - (APPROACH_RADIUS * cos(approach_angle));
                        double y = green.y - (APPROACH_RADIUS * sin(approach_angle));

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

                        while(srv.response.plan.poses.size() == 0){
                            approach_angle = fmod(rand(), 2*PI);
                            Goal.pose.position.x = green.x - (APPROACH_RADIUS * cos(approach_angle));
                            Goal.pose.position.y = green.y - (APPROACH_RADIUS * sin(approach_angle));

                            desired_yaw = constrainAngle(approach_angle - PI/2);

                            cy = cos(desired_yaw * 0.5);
                            sy = sin(desired_yaw * 0.5);

                            Goal.pose.orientation.z = cy;
                            Goal.pose.orientation.w = sy;

                            srv.request.goal = Goal;
                            ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
                        }

                        ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

                        goal.target_pose.pose = Goal.pose;

                        ROS_INFO("Sending goal");
                        ac.sendGoal(goal);

                        while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                            bool avoided_collision = false;
                            bool check_collision = collision_check(yaw, pose, goal, ac, collision_radius);

                            while(check_collision){
                                check_collision = collision_check(yaw, pose, goal, ac, collision_radius);
                                avoided_collision = true;
                                ros::spinOnce();
                                pose = get_robot_pos(listener, yaw);
                            }

                            if(avoided_collision){
                                ac.cancelGoal();
                                ac.sendGoal(goal);
                            }
                        }

                        //drop
                        int puck_counter = 0;

                        for(int i = 0; i < green_pucks; i++, puck_counter++){
                            eurobot2019_messages::drop_command drop_command_msg;
                            drop_command_msg.left = 1;
                            drop_interface.set_msg(drop_command_msg);
                            std_msgs::Int32 drop_status_msg;
                            ros::spinOnce();
                            drop_status_msg = drop_interface.get_msg();

                            while(drop_status_msg.data > 0){
                                ros::spinOnce();
                                drop_status_msg = drop_interface.get_msg();
                            }
                        }

                        score += puck_counter * 6;

                        green_pucks -= puck_counter;
                    }

                    if(red_pucks > 0){
                        geometry_msgs::PoseStamped Start;
                        geometry_msgs::PoseStamped Goal;

                        Start.header.seq = 0;
                        Start.header.stamp = ros::Time::now();
                        Start.header.frame_id = "/map";
                        Start.pose = pose;

                        double approach_angle = atan2((red.y - pose.position.y), (red.x - pose.position.x));
                        double x = red.x - (APPROACH_RADIUS * cos(approach_angle));
                        double y = red.y - (APPROACH_RADIUS * sin(approach_angle));

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

                        while(srv.response.plan.poses.size() == 0){
                            approach_angle = fmod(rand(), 2*PI);
                            Goal.pose.position.x = red.x - (APPROACH_RADIUS * cos(approach_angle));
                            Goal.pose.position.y = red.y - (APPROACH_RADIUS * sin(approach_angle));

                            desired_yaw = constrainAngle(approach_angle - PI/2);

                            cy = cos(desired_yaw * 0.5);
                            sy = sin(desired_yaw * 0.5);

                            Goal.pose.orientation.z = cy;
                            Goal.pose.orientation.w = sy;

                            srv.request.goal = Goal;
                            ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
                        }

                        ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

                        goal.target_pose.pose = Goal.pose;

                        ROS_INFO("Sending goal");
                        ac.sendGoal(goal);

                        while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                            bool avoided_collision = false;
                            bool check_collision = collision_check(yaw, pose, goal, ac, collision_radius);

                            while(check_collision){
                                check_collision = collision_check(yaw, pose, goal, ac, collision_radius);
                                avoided_collision = true;
                                ros::spinOnce();
                                pose = get_robot_pos(listener, yaw);
                            }

                            if(avoided_collision){
                                ac.cancelGoal();
                                ac.sendGoal(goal);
                            }
                        }

                        int puck_counter = 0;

                        for(int i = 0; i < red_pucks; i++, puck_counter++){
                            eurobot2019_messages::drop_command drop_command_msg;
                            drop_command_msg.right = 1;
                            drop_interface.set_msg(drop_command_msg);
                            std_msgs::Int32 drop_status_msg;
                            ros::spinOnce();
                            drop_status_msg = drop_interface.get_msg();

                            while(drop_status_msg.data > 0){
                                ros::spinOnce();
                                drop_status_msg = drop_interface.get_msg();
                            }
                        }

                        score += puck_counter * 6;

                        red_pucks -= puck_counter;
                    }

                    //head to next puck
                    std::vector <double> puck_score;
                    pose = get_robot_pos(listener, yaw);
                    ros::spinOnce();
                    for(int i = 0; i < poses.poses.size(); i++){
                        double puck_distance = pow(poses.poses[i].position.x - pose.position.x, 2) + pow(poses.poses[i].position.y - pose.position.y, 2);

                        int puck_value;

                        if ((pucks_color[i] == 2) || (pucks_color[i] == 1)){
                          puck_value = 6;//value for green;
                        }

                        else{
                          puck_value = 0;
                        }
                        puck_score.push_back(puck_value/puck_distance);//Scale by a constant?
                    }

                  int chosen_puck = std::max_element(puck_score.begin(),puck_score.end()) - puck_score.begin();
                  geometry_msgs::Pose puck = poses.poses[chosen_puck];
                  current_puck_colour = pucks_color[chosen_puck];
                  is_vertical = isVertical[chosen_puck];

                  geometry_msgs::PoseStamped Start;
                  geometry_msgs::PoseStamped Goal;

                  Start.header.seq = 0;
                  Start.header.stamp = ros::Time::now();
                  Start.header.frame_id = "/map";
                  Start.pose = pose;

                  double approach_angle = atan2((puck.position.y - pose.position.y), (puck.position.x - pose.position.x));
                  double x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
                  double y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

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

                  while(srv.response.plan.poses.size() == 0){
                      approach_angle = fmod(rand(), 2*PI);
                      Goal.pose.position.x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
                      Goal.pose.position.y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

                      desired_yaw = constrainAngle(approach_angle - PI/2);

                      cy = cos(desired_yaw * 0.5);
                      sy = sin(desired_yaw * 0.5);

                      Goal.pose.orientation.z = cy;
                      Goal.pose.orientation.w = sy;

                      srv.request.goal = Goal;
                      ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
                  }

                  ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

                  goal.target_pose.pose = Goal.pose;
                }

                else{
                    //head to next puck
                    std::vector <double> puck_score;
                    pose = get_robot_pos(listener, yaw);
                    ros::spinOnce();
                    for(int i = 0; i < poses.poses.size(); i++){
                        double puck_distance = pow(poses.poses[i].position.x - pose.position.x, 2) + pow(poses.poses[i].position.y - pose.position.y, 2);

                        int puck_value;

                        if ((pucks_color[i] == 2) || (pucks_color[i] == 1)){
                          puck_value = 6;//value for green;
                        }

                        else{
                          puck_value = 0;
                        }
                        puck_score.push_back(puck_value/puck_distance);//Scale by a constant?
                    }

                  int chosen_puck = std::max_element(puck_score.begin(),puck_score.end()) - puck_score.begin();
                  geometry_msgs::Pose puck = poses.poses[chosen_puck];
                  current_puck_colour = pucks_color[chosen_puck];
                  is_vertical = isVertical[chosen_puck];

                  geometry_msgs::PoseStamped Start;
                  geometry_msgs::PoseStamped Goal;

                  Start.header.seq = 0;
                  Start.header.stamp = ros::Time::now();
                  Start.header.frame_id = "/map";
                  Start.pose = pose;

                  double approach_angle = atan2((puck.position.y - pose.position.y), (puck.position.x - pose.position.x));
                  double x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
                  double y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

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

                  while(srv.response.plan.poses.size() == 0){
                      approach_angle = fmod(rand(), 2*PI);
                      Goal.pose.position.x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
                      Goal.pose.position.y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

                      desired_yaw = constrainAngle(approach_angle - PI/2);

                      cy = cos(desired_yaw * 0.5);
                      sy = sin(desired_yaw * 0.5);

                      Goal.pose.orientation.z = cy;
                      Goal.pose.orientation.w = sy;

                      srv.request.goal = Goal;
                      ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
                  }

                  ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

                  goal.target_pose.pose = Goal.pose;
                }
            }

            else{
                //Set goal next puck
                std::vector <double> puck_score;
                pose = get_robot_pos(listener, yaw);
                ros::spinOnce();
                for(int i = 0; i < poses.poses.size(); i++){
                    double puck_distance = pow(poses.poses[i].position.x - pose.position.x, 2) + pow(poses.poses[i].position.y - pose.position.y, 2);

                    int puck_value;

                    if (pucks_color[i] == 2){
                      puck_value = 8;//value for green;
                    }
                    else if (pucks_color[i] == 3){
                      puck_value = 12;//value for blue;
                    }
                    else{
                      puck_value = 0;
                    }
                    puck_score.push_back(puck_value/puck_distance);//Scale by a constant?
                }

              int chosen_puck = std::max_element(puck_score.begin(),puck_score.end()) - puck_score.begin();
              geometry_msgs::Pose puck = poses.poses[chosen_puck];
              current_puck_colour = pucks_color[chosen_puck];
              is_vertical = isVertical[chosen_puck];

              geometry_msgs::PoseStamped Start;
              geometry_msgs::PoseStamped Goal;

              Start.header.seq = 0;
              Start.header.stamp = ros::Time::now();
              Start.header.frame_id = "/map";
              Start.pose = pose;

              double approach_angle = atan2((puck.position.y - pose.position.y), (puck.position.x - pose.position.x));
              double x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
              double y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

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

              while(srv.response.plan.poses.size() == 0){
                  approach_angle = fmod(rand(), 2*PI);
                  Goal.pose.position.x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
                  Goal.pose.position.y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

                  desired_yaw = constrainAngle(approach_angle - PI/2);

                  cy = cos(desired_yaw * 0.5);
                  sy = sin(desired_yaw * 0.5);

                  Goal.pose.orientation.z = cy;
                  Goal.pose.orientation.w = sy;

                  srv.request.goal = Goal;
                  ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
              }

              ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

              goal.target_pose.pose = Goal.pose;
            }
        }

        else if(heading_to_ramp){
            heading_to_ramp = false;
            final_stage = true;

            int puck_counter = 0;

            for(int i = 0; i < blue_pucks && puck_counter <= 4; i++, puck_counter++){
                eurobot2019_messages::drop_command drop_command_msg;
                drop_command_msg.right = 1;
                drop_interface.set_msg(drop_command_msg);
                std_msgs::Int32 drop_status_msg;
                ros::spinOnce();
                drop_status_msg = drop_interface.get_msg();

                while(drop_status_msg.data > 0){
                    ros::spinOnce();
                    drop_status_msg = drop_interface.get_msg();
                }
            }

            score += puck_counter * 12;

            blue_pucks -= puck_counter;

            int puck_limit = 4 - puck_counter;

            puck_counter = 0;

            for(int i = 0; i < green_pucks && puck_counter <= puck_limit; i++, puck_counter++){
                eurobot2019_messages::drop_command drop_command_msg;
                drop_command_msg.left = 1;
                drop_interface.set_msg(drop_command_msg);
                std_msgs::Int32 drop_status_msg;
                ros::spinOnce();
                drop_status_msg = drop_interface.get_msg();

                while(drop_status_msg.data > 0){
                    ros::spinOnce();
                    drop_status_msg = drop_interface.get_msg();
                }
            }

            score += (puck_counter + 1) * 8;

            green_pucks -= puck_counter;

            pucks_taken = green_pucks + blue_pucks;

            //head to next puck
            std::vector <double> puck_score;
            pose = get_robot_pos(listener, yaw);
            ros::spinOnce();
            for(int i = 0; i < poses.poses.size(); i++){
                double puck_distance = pow(poses.poses[i].position.x - pose.position.x, 2) + pow(poses.poses[i].position.y - pose.position.y, 2);

                int puck_value;

                if ((pucks_color[i] == 2) || (pucks_color[i] == 1)){
                  puck_value = 6;//value for green;
                }

                else{
                  puck_value = 0;
                }
                puck_score.push_back(puck_value/puck_distance);//Scale by a constant?
            }

          int chosen_puck = std::max_element(puck_score.begin(),puck_score.end()) - puck_score.begin();
          geometry_msgs::Pose puck = poses.poses[chosen_puck];
          current_puck_colour = pucks_color[chosen_puck];
          is_vertical = isVertical[chosen_puck];

          geometry_msgs::PoseStamped Start;
          geometry_msgs::PoseStamped Goal;

          Start.header.seq = 0;
          Start.header.stamp = ros::Time::now();
          Start.header.frame_id = "/map";
          Start.pose = pose;

          double approach_angle = atan2((puck.position.y - pose.position.y), (puck.position.x - pose.position.x));
          double x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
          double y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

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

          while(srv.response.plan.poses.size() == 0){
              approach_angle = fmod(rand(), 2*PI);
              Goal.pose.position.x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
              Goal.pose.position.y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

              desired_yaw = constrainAngle(approach_angle - PI/2);

              cy = cos(desired_yaw * 0.5);
              sy = sin(desired_yaw * 0.5);

              Goal.pose.orientation.z = cy;
              Goal.pose.orientation.w = sy;

              srv.request.goal = Goal;
              ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
          }

          ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

          goal.target_pose.pose = Goal.pose;
        }

        else{
            ROS_INFO("The base failed to move to next position and orientation");
            //Set goal another puck
            //Assume receiving geometry_msgs::PoseArray poses, array of ints containing colours int* pucks_color[], array of bools containing whether or not is_vertical called isVertical[]

            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> total_time_span = total_time - t1;
            if((((pucks_taken < 4 && total_time_span.count()) > 80000) || pucks_taken >= 3) && final_stage){
                //Go back and deposit, instead of finding another puck
                if(green_pucks > 0){
                    geometry_msgs::PoseStamped Start;
                    geometry_msgs::PoseStamped Goal;

                    Start.header.seq = 0;
                    Start.header.stamp = ros::Time::now();
                    Start.header.frame_id = "/map";
                    Start.pose = pose;

                    double approach_angle = atan2((green.y - pose.position.y), (green.x - pose.position.x));
                    double x = green.x - (APPROACH_RADIUS * cos(approach_angle));
                    double y = green.y - (APPROACH_RADIUS * sin(approach_angle));

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

                    while(srv.response.plan.poses.size() == 0){
                        approach_angle = fmod(rand(), 2*PI);
                        Goal.pose.position.x = green.x - (APPROACH_RADIUS * cos(approach_angle));
                        Goal.pose.position.y = green.y - (APPROACH_RADIUS * sin(approach_angle));

                        desired_yaw = constrainAngle(approach_angle - PI/2);

                        cy = cos(desired_yaw * 0.5);
                        sy = sin(desired_yaw * 0.5);

                        Goal.pose.orientation.z = cy;
                        Goal.pose.orientation.w = sy;

                        srv.request.goal = Goal;
                        ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
                    }

                    ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

                    goal.target_pose.pose = Goal.pose;

                    ROS_INFO("Sending goal");
                    ac.sendGoal(goal);

                    while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                        bool avoided_collision = false;

                        bool check_collision = collision_check(yaw, pose, goal, ac, collision_radius);

                        while(check_collision){
                            check_collision = collision_check(yaw, pose, goal, ac, collision_radius);
                            avoided_collision = true;
                            ros::spinOnce();
                            pose = get_robot_pos(listener, yaw);
                        }

                        if(avoided_collision){
                            ac.cancelGoal();
                            ac.sendGoal(goal);
                        }
                    }

                    //drop
                    int puck_counter = 0;

                    for(int i = 0; i < green_pucks; i++, puck_counter++){
                        eurobot2019_messages::drop_command drop_command_msg;
                        drop_command_msg.left = 1;
                        drop_interface.set_msg(drop_command_msg);
                        std_msgs::Int32 drop_status_msg;
                        ros::spinOnce();
                        drop_status_msg = drop_interface.get_msg();

                        while(drop_status_msg.data > 0){
                            ros::spinOnce();
                            drop_status_msg = drop_interface.get_msg();
                        }
                    }

                    score += puck_counter * 6;

                    green_pucks -= puck_counter;
                }

                if(red_pucks > 0){
                    geometry_msgs::PoseStamped Start;
                    geometry_msgs::PoseStamped Goal;

                    Start.header.seq = 0;
                    Start.header.stamp = ros::Time::now();
                    Start.header.frame_id = "/map";
                    Start.pose = pose;

                    double approach_angle = atan2((red.y - pose.position.y), (red.x - pose.position.x));
                    double x = red.x - (APPROACH_RADIUS * cos(approach_angle));
                    double y = red.y - (APPROACH_RADIUS * sin(approach_angle));

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

                    while(srv.response.plan.poses.size() == 0){
                        approach_angle = fmod(rand(), 2*PI);
                        Goal.pose.position.x = red.x - (APPROACH_RADIUS * cos(approach_angle));
                        Goal.pose.position.y = red.y - (APPROACH_RADIUS * sin(approach_angle));

                        desired_yaw = constrainAngle(approach_angle - PI/2);

                        cy = cos(desired_yaw * 0.5);
                        sy = sin(desired_yaw * 0.5);

                        Goal.pose.orientation.z = cy;
                        Goal.pose.orientation.w = sy;

                        srv.request.goal = Goal;
                        ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
                    }

                    ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

                    goal.target_pose.pose = Goal.pose;

                    ROS_INFO("Sending goal");
                    ac.sendGoal(goal);

                    while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                        bool avoided_collision = false;

                        bool check_collision = collision_check(yaw, pose, goal, ac, collision_radius);

                        while(check_collision){
                            check_collision = collision_check(yaw, pose, goal, ac, collision_radius);
                            avoided_collision = true;
                            ros::spinOnce();
                            pose = get_robot_pos(listener, yaw);
                        }

                        if(avoided_collision){
                            ac.cancelGoal();
                            ac.sendGoal(goal);
                        }
                    }

                    //drop
                    int puck_counter = 0;

                    for(int i = 0; i < red_pucks; i++, puck_counter++){
                        eurobot2019_messages::drop_command drop_command_msg;
                        drop_command_msg.right = 1;
                        drop_interface.set_msg(drop_command_msg);
                        std_msgs::Int32 drop_status_msg;
                        ros::spinOnce();
                        drop_status_msg = drop_interface.get_msg();

                        while(drop_status_msg.data > 0){
                            ros::spinOnce();
                            drop_status_msg = drop_interface.get_msg();
                        }
                    }

                    score += puck_counter * 6;

                    red_pucks -= puck_counter;
                }


                //Then, find another puck
                std::vector <double> puck_score;
                pose = get_robot_pos(listener, yaw);
                ros::spinOnce();
                for(int i = 0; i < poses.poses.size(); i++){
                    double puck_distance = pow(poses.poses[i].position.x - pose.position.x, 2) + pow(poses.poses[i].position.y - pose.position.y, 2);

                    int puck_value;

                    if ((pucks_color[i] == 2) || (pucks_color[i] == 1)){
                      puck_value = 6;//value for green;
                    }

                    else{
                      puck_value = 0;
                    }
                    puck_score.push_back(puck_value/puck_distance);//Scale by a constant?
                }

              int chosen_puck = std::max_element(puck_score.begin(),puck_score.end()) - puck_score.begin();
              geometry_msgs::Pose puck = poses.poses[chosen_puck];
              current_puck_colour = pucks_color[chosen_puck];
              is_vertical = isVertical[chosen_puck];

              geometry_msgs::PoseStamped Start;
              geometry_msgs::PoseStamped Goal;

              Start.header.seq = 0;
              Start.header.stamp = ros::Time::now();
              Start.header.frame_id = "/map";
              Start.pose = pose;

              double approach_angle = atan2((puck.position.y - pose.position.y), (puck.position.x - pose.position.x));
              double x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
              double y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

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

              while(srv.response.plan.poses.size() == 0){
                  approach_angle = fmod(rand(), 2*PI);
                  Goal.pose.position.x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
                  Goal.pose.position.y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

                  desired_yaw = constrainAngle(approach_angle - PI/2);

                  cy = cos(desired_yaw * 0.5);
                  sy = sin(desired_yaw * 0.5);

                  Goal.pose.orientation.z = cy;
                  Goal.pose.orientation.w = sy;

                  srv.request.goal = Goal;
                  ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
              }

              ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

              goal.target_pose.pose = Goal.pose;
            }

            else if(final_stage){
                //find another puck
                std::vector <double> puck_score;
                pose = get_robot_pos(listener, yaw);
                ros::spinOnce();
                for(int i = 0; i < poses.poses.size(); i++){
                    double puck_distance = pow(poses.poses[i].position.x - pose.position.x, 2) + pow(poses.poses[i].position.y - pose.position.y, 2);

                    int puck_value;

                    if ((pucks_color[i] == 2) || (pucks_color[i] == 1)){
                      puck_value = 6;//value for green;
                    }

                    else{
                      puck_value = 0;
                    }
                    puck_score.push_back(puck_value/puck_distance);//Scale by a constant?
                }

              int chosen_puck = std::max_element(puck_score.begin(),puck_score.end()) - puck_score.begin();
              geometry_msgs::Pose puck = poses.poses[chosen_puck];
              current_puck_colour = pucks_color[chosen_puck];
              is_vertical = isVertical[chosen_puck];

              geometry_msgs::PoseStamped Start;
              geometry_msgs::PoseStamped Goal;

              Start.header.seq = 0;
              Start.header.stamp = ros::Time::now();
              Start.header.frame_id = "/map";
              Start.pose = pose;

              double approach_angle = atan2((puck.position.y - pose.position.y), (puck.position.x - pose.position.x));
              double x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
              double y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

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

              while(srv.response.plan.poses.size() == 0){
                  approach_angle = fmod(rand(), 2*PI);
                  Goal.pose.position.x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
                  Goal.pose.position.y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

                  desired_yaw = constrainAngle(approach_angle - PI/2);

                  cy = cos(desired_yaw * 0.5);
                  sy = sin(desired_yaw * 0.5);

                  Goal.pose.orientation.z = cy;
                  Goal.pose.orientation.w = sy;

                  srv.request.goal = Goal;
                  ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
              }

              ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

              goal.target_pose.pose = Goal.pose;
            }

            else{
                std::vector <double> puck_score;
                pose = get_robot_pos(listener, yaw);
                ros::spinOnce();
                for(int i = 0; i < poses.poses.size(); i++){
                double puck_distance = pow(poses.poses[i].position.x - pose.position.x, 2) + pow(poses.poses[i].position.y - pose.position.y, 2);

                double proximity_to_prev_goal = pow(poses.poses[i].position.x - goal.target_pose.pose.position.x, 2) + pow(poses.poses[i].position.y - goal.target_pose.pose.position.y, 2);

                int puck_value;

                if (pucks_color[i] == 2){
                  puck_value = 8;//value for green;
                }
                else if (pucks_color[i] == 3){
                  puck_value = 12;//value for blue;
                }
                else{
                  puck_value = 0;
                }
                puck_score.push_back((puck_value  * pow(proximity_to_prev_goal, (total_collision_time/1000)))/puck_distance);//Scale by a constant?
              }

              int chosen_puck = std::max_element(puck_score.begin(),puck_score.end()) - puck_score.begin();
              geometry_msgs::Pose puck = poses.poses[chosen_puck];
              current_puck_colour = pucks_color[chosen_puck];
              is_vertical = isVertical[chosen_puck];

              geometry_msgs::PoseStamped Start;
              geometry_msgs::PoseStamped Goal;

              Start.header.seq = 0;
              Start.header.stamp = ros::Time::now();
              Start.header.frame_id = "/map";
              Start.pose = pose;

              double x;
              double y;
              double approach_angle;

              double desired_yaw;

              double cy;
              double sy;

              if(is_vertical){
                x = puck.position.x + APPROACH_RADIUS;
                y = puck.position.y;

                desired_yaw = PI/2;

                cy = cos(desired_yaw * 0.5);
                sy = sin(desired_yaw * 0.5);
              }

              else {
                approach_angle = atan2((puck.position.y - pose.position.y), (puck.position.x - pose.position.x));
                x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
                y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

                desired_yaw = constrainAngle(approach_angle - PI/2);

                cy = cos(desired_yaw * 0.5);
                sy = sin(desired_yaw * 0.5);
              }


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

              while(srv.response.plan.poses.size() == 0){
                  approach_angle = fmod(rand(), 2*PI);
                  Goal.pose.position.x = puck.position.x - (APPROACH_RADIUS * cos(approach_angle));
                  Goal.pose.position.y = puck.position.y - (APPROACH_RADIUS * sin(approach_angle));

                  desired_yaw = constrainAngle(approach_angle - PI/2);

                  cy = cos(desired_yaw * 0.5);
                  sy = sin(desired_yaw * 0.5);

                  Goal.pose.orientation.z = cy;
                  Goal.pose.orientation.w = sy;

                  srv.request.goal = Goal;
                  ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
              }

              ROS_INFO("Plan size: %d", srv.response.plan.poses.size());

              goal.target_pose.pose = Goal.pose;
            }
        }

        ros::spinOnce();

        sleeper.sleep();
    }
}
*/
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

void wallignore(double yaw, std::vector<int>& wall_vector, geometry_msgs::Pose pose, double collision_radius){
    // Might need to reconsider if pucks are taken to be occupied space
        //relative to origin (edge of map), not (0,0) of map
    double resolution = map_metadata.resolution;
    int width = map_metadata.width;
    int height = map_metadata.height;
    geometry_msgs::Pose origin = map_metadata.origin;

    int right;
    int left;
    int top;
    int bottom;

    double base_link_x = pose.position.x - origin.position.x;
    double base_link_y = pose.position.y - origin.position.y;
    int base_link_pixel_x = round((pose.position.x - origin.position.x)/resolution);
    int base_link_pixel_y = round((pose.position.y - origin.position.y)/resolution);

    std::vector<bool> bool_wall_vector = {0, 0};

    if ( (base_link_y + collision_radius) > resolution * height){
        top = height;
    }

    else{
        top = ceil((base_link_y + collision_radius)/resolution);
    }

    if ( (base_link_x + collision_radius) > resolution * width){
        right = width;
    }

    else{
        right = ceil((base_link_x + collision_radius)/resolution);
    }

    if ( (base_link_x - collision_radius) < 0){
        left = 0;
    }

    else{
        left = floor((base_link_x - collision_radius)/resolution);
    }

    if ( (base_link_y - collision_radius) < 0){
        bottom = 0;
    }

    else{
        bottom = ceil((base_link_y - collision_radius)/resolution);
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

    for(int i = 0; i < 2; i++){
        if(bool_wall_vector[i]){
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
  else if (((0.3926991 <= angler2w) && (angler2w <= (-5.1050881 + 3.1415926*2))) || (((0.3926991 + 3.1415926*2) <= angler2w) && (angler2w <= (-5.1050881 + 3.1415926*4)))){
    // Suppress collision_avoidance[1]
    wall_vector_instance.push_back(0);
  }
  else if (((-1.1780972 <= angler2w) && (angler2w <= -0.3926991)) || (((-1.1780972 + 3.1415926*2) <= angler2w) && (angler2w <= (-0.3926991 + 3.1415926*2)))){
    // Suppress collision_avoidance[7]
    wall_vector_instance.push_back(0);
  }
  if (((-4.3196899 <= angler2w) && (angler2w <= -3.5342917)) || (((-4.3196899 + 3.1415926*2) <= angler2w) && (angler2w <= (-3.5342917 + 3.1415926*2)))){
    // Suppress collision_avoidance[3]
    wall_vector_instance.push_back(1);
  }
  else if (((-3.5342917 <= angler2w) && (angler2w <= -2.7488936)) || (((-3.5342917 + 3.1415926*2) <= angler2w) && (angler2w <= (-2.7488936 + 3.1415926*2)))){
    // Suppress collision_avoidance[4]
    wall_vector_instance.push_back(1);
  }
  else if (((-2.7488936 <= angler2w) && (angler2w <= -1.9634954)) || (((-2.7488936 + 3.1415926*2) <= angler2w) && (angler2w <= (-1.9634954 + 3.1415926*2)))){
    // Suppress collision_avoidance[5]
    wall_vector_instance.push_back(1);
  }
}

geometry_msgs::Pose get_robot_pos(tf::TransformListener& listener, double& yaw){
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

      // construct a pose message
      geometry_msgs::Pose pose;

      pose.orientation.x = transform.getRotation().getX();
      pose.orientation.y = transform.getRotation().getY();
      pose.orientation.z = transform.getRotation().getZ();
      pose.orientation.w = transform.getRotation().getW();

      pose.position.x = transform.getOrigin().getX();
      pose.position.y = transform.getOrigin().getY();
      pose.position.z = transform.getOrigin().getZ();

      yaw = tf::getYaw(transform.getRotation());

      return pose;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }
}

double space_distance(double  yaw, geometry_msgs::Pose pose, int direction){
    double resolution = map_metadata.resolution;
    int width = map_metadata.width;
    int height = map_metadata.height;
    geometry_msgs::Pose origin = map_metadata.origin;

    double base_link_x = pose.position.x - map_metadata.origin.position.x;
    double base_link_y = pose.position.y - map_metadata.origin.position.y;
    double motion_angle = yaw - (direction * PI/4) + PI/2;
    int base_link_pixel_x = round((base_link_x)/resolution);
    int base_link_pixel_y = round((base_link_y)/resolution);
    bool found_occupied = false;

    double distance_to_occupied_squared;

      for(int i = base_link_pixel_x; i >= 0 && i <= width && found_occupied; i++){
          base_link_x += resolution;
          base_link_y += resolution * tan(motion_angle);

          int pixel_x = round((base_link_x)/resolution);
          int pixel_y = round((base_link_y)/resolution);

          if(pixel_x <= 0 || pixel_y <= 0 || pixel_x >= width || pixel_y >= height){
              found_occupied = true;
              distance_to_occupied_squared = pow((base_link_x - (pose.position.x - map_metadata.origin.position.x)), 2) + pow((base_link_y - (pose.position.y - map_metadata.origin.position.y)), 2);
          }

          else{
              if(map.data[pixel_x + pixel_y * width] == 100){
                  found_occupied = true;
                  distance_to_occupied_squared = pow((base_link_x - (pose.position.x - map_metadata.origin.position.x)), 2) + pow((base_link_y - (pose.position.y - map_metadata.origin.position.y)), 2);
              }
          }
      }

      return distance_to_occupied_squared;
}

bool collision_check(double yaw, geometry_msgs::Pose pose, double collision_radius){

bool blocked_by_non_wall = false;
//std::vector<int> blocked_directions;
//blocked_directions.push_back(15); //junk value to push

std::vector<int> wall_vector;
wallignore(yaw, wall_vector, pose, collision_radius);

for(int i = 0, j = 0; i < 2; i++){
  if (i == wall_vector[j]){
    //blocked_directions.push_back(i);

    if (j < wall_vector.size()){
      j++;
    }
  }

  else{
    if(collision_avoidance.data[i]){
      blocked_by_non_wall = true;
      //blocked_directions.push_back(i);
    }
  }
}

return blocked_by_non_wall;
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

// Returns true when puck is successfully grabbed
// Non-blocking
bool ready_to_grab(bool is_vertical) {
    auto grabber_status = grabber_interface.get_message();
    std_msgs::Int8MultiArray new_msg;
    if(!grabber_status.data) return false;
    if(is_vertical) {
        if(grabber_status.data[0] != 1 
         || grabber_status.data[1] != 1
         || grabber_status.data[2] != 0
         || grabber_status.data[3] != 2) {
            grabber_msg_data = {1, 1, 0, 2};
            new_msg.data = grabber_msg_data;
            grabber_interface.set_message(new_msg);
            return false;
        }
    }
    else {
        if(grabber_status.data[0] != 0
         || grabber_status.data[1] != 0
         || grabber_status.data[2] != 0
         || grabber_status.data[3] != 2) {
            grabber_msg_data = {0, 0, 0, 2};
            new_msg.data=grabber_msg_data;
            grabber_interface.set_message(new_msg);
            return false;
        }
    }
    return true; 
}

bool grab_puck(bool is_vertical) {
    if(!ready_to_grab(is_vertical)) {
        auto grabber_status = grabber_interface.get_message();
        if((grabber_status.data[0] == 2 && is_vertical))
            return true;
        return false;
    }
    auto grabber_status = grabber_interface.get_message();
    if(is_vertical) {
        if(grabber_status.data[2] != 2) {
            grabber_status.data[2] = 2;
            grabber_interface.set_msg(grabber_status);
            return false;
        }
    }
    else {
        if(grabber_status.data[2] != 1) {
            grabber_status.data[2] = 1;
            grabber_interface.set_msg(grabber_status);
            return false;
        }
    }
    return true;
}

bool ready_to_drop() {
    auto grabber_status = grabber_interface.get_message();
    std_msgs::Int8MultiArray new_msg;
    if(grabber_status.data[0] != 2
     || grabber_status.data[1] != 1
     || grabber_status.data[2] != 2
     || grabber_status.data[3] != 2) {
        grabber_msg_data = {2, 1, 2, 2};
        new_msg.data = grabber_msg_data;
        grabber_interface.set_message(new_msg);
        return false;
    }
    return true;
}

// Blocking drop puck
void drop_puck() {
    if(!ready_to_drop()) return false;
    auto grabber_status = grabber_interface.get_message();
    do {
        grabber_status = grabber_interface.get_message();
        std_msgs::Int8MultiArray new_msg;
        grabber_msg_data = grabber_status.data;
        grabber_msg_data[2] = 0;
        new_msg.data = grabber_msg_data;
        grabber_interface.set_msg(new_msg);
    }
    while(grabber_status.data[2] != 0);
    while(grabber_status.data[3] != flipper) {
        std_msgs::Int8MultiArray new_msg;
        grabber_msg_data = grabber_status.data;
        grabber_msg_data[3] = flipper;
        new_msg.data = grabber_msg_data;
        grabber_interface.set_msg(new_msg);
    }
    std_msgs::Int8MultiArray new_msg;
    grabber_msg_data = {0, 1, 0, 2};
    new_msg.data = grabber_msg_data;
    grabber_interface.set_msg(new_msg);
}

