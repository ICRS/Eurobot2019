/////////////////////////////////////////////////////////////////////////////////////////////////////////
//THIS CODE BLOCK IS OUT OF PLACE, SHOULD BE USED TO FIND A GOAL, GIVEN THE POSITION OF A PUCK

//geometry_msgs::Pose puck, contains position/orientation of puck
//geometry_msgs::Pose pose, contains position/orientation of robot


# define APPROACH_RADIUS 3 //distance away from puck robot must be at, to pick it up

// puck - pose = vector for direction/pose_orientation
// use make_plan to check that place is free




// CODE BLOCK ENDS HERE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//THIS CODE BLOCK IS OUT OF PLACE, SHOULD BE USED TO AVOID COLLISIONS

// assume object of type eurobot2019_messages::collision_avoidance (array of bools), called collision_avoidance
// assume also vector of array elements that correspond with a wall, called wall_vector

bool collision_check(double yaw, geometry_msg::Pose pose, geometry_msg::Pose goal){

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
  //ignore directions between two blocked_directions, unless super close
  std::vector<double> scores;
  std::vector<int> directions;

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
          //find distance/direction and rest of score
          //If not, abandon
        }


        else if(blocked_directions[1] == 1 || blocked_directions[blocked_directions.size() - 1] == 7){
          //adjust score accordingly
        }

        else{
          //none of them are, no score penalty
          //find rest of score, i.e. space(distance to closest occupied point in direction), angle from goal
        }
      }

      if(i == 7){
        if(blocked_directions[1] == 0 && blocked_directions[blocked_directions.size() - 1] == 6){
          //find distance/direction and score
          //If not, abandon
        }


        if(blocked_directions[1] == 0 || blocked_directions[blocked_directions.size() - 1] == 6){
          //adjust score accordingly
        }

        else{
          //none of them are, no score penalty
          //find rest of score, i.e. space(distance to closest occupied point in direction), angle from goal
        }
      }

      else{
        if(blocked_directions[r] == i - 1 && blocked_directions[r + 1] == i + 1){
            if (pow(goal.target_pose.pose.position.x - pose.pose.position.x, 2) + pow(goal.target_pose.pose.position.y - pose.pose.position.y - , 2)) <= CLOSE_ENOUGH){
                vector = wallignore_instance(goal.target_pose.pose.position.y - pose.pose.position.y, goal.target_pose.pose.position.x - pose.pose.position.x, yaw)
                for(int k = 0; k < vector.size(); k++){
                    if(vector[k] == i){
                        //score
                        break;
                    }
                }
            }

            else{
                //ignore
            }
        }

        if(blocked_directions[r] == i - 1 || blocked_directions[r + 1] == i + 1){
          //adjust score accordingly
          // Calculate rest of score
        }

        else{
            pose = get_robot_pos(listener, yaw);
            double base_link_x = pose.pose.position.x - origin.position.x;
            double base_link_y = pose.pose.position.y - origin.position.y;
            double motion_angle = yaw - (i * PI/4) + PI/2;
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
                      double distance_to_occupied = pow((base_link_x - (pose.pose.position.x - origin.position.x)), 2) + pow((base_link_y - (pose.pose.position.y - origin.position.y)), 2)
                  }

                  else{
                      if(map.data[pixel_x + pixel_y * width] == 100){
                          found_occupied = true;
                          double distance_to_occupied = pow((base_link_x - (pose.pose.position.x - origin.position.x)), 2) + pow((base_link_y - (pose.pose.position.y - origin.position.y)), 2)
                      }
                  }
              }
            //none of them are, no score penalty
            //find rest of score, i.e. space(distance to closest occupied point in direction), angle from goal
          }
        }
      }
    }
  }

  return blocked_by_non_wall;
}




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
#include <cmath>
#include "message_interface.hpp"

#define CLOSE_ENOUGH //When goal is close enough, such that we can ignore collision_avoidance around adjacent directions adjacent to goal direction
#define PI 3.1415926

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
double anglescores(double yaw, double x, double y);
double constrainAngle(double x);
Quaterniond toQuaternion(double yaw, double pitch, double roll);
bool collision_check(double yaw, geometry_msg::Pose pose, geometry_msg::Pose goal);

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
    ros::Subscriber sub_collision_avoidance = n.subscribe("collision_avoidance", 1, sub_collision_avoidance_Callback);

    do{
        nh.spinOnce();
    }while(!got_map && !got_map_metadata);

    double resolution = map_metadata.resolution;
    int width = map_metadata.width;
    int height = map_metadata.height;
    geometry_msg::Pose origin = map_metadeta.origin;

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

    //Assume first goal is known?
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.y = -1.0;

    ros::Rate sleeper(100);

    while (ros::ok()){
        //Set next target, has been set?

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        //ac.cancelGoal();
        ac.waitForResult();

        static auto t1 = std::chrono::high_resolution_clock::now();
        bool has_moved_closer = true;
        geometry_msgs::Pose pose = get_robot_pos(listener, yaw);
        double prev_distance = pow(pose.pose.position.x - goal.target_pose.pose.position.x, 2) + pow(pose.pose.position.y - goal.target_pose.pose.position.y, 2);

        while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || has_moved_closer) {//check collision avoidance also
            bool avoided_collision = false;
            geometry_msgs::Pose pose = get_robot_pos(listener, yaw);
            double distance = pow(pose.pose.position.x - goal.target_pose.pose.position.x, 2) + pow(pose.pose.position.y - goal.target_pose.pose.position.y, 2);
            nh.spinOnce();
            //check collision avoidance
            while(collision_check(yaw, pose, goal.target_pose)){
                avoided_collision = true;
                nh.spinOnce();
                geometry_msgs::Pose pose = get_robot_pos(listener, yaw);
            }

            if(avoided_collision){
                //reset goal
                //
            }

            else if(distance < prev_distance){
                prev_distance = distance;
                t1 = std::chrono::high_resolution_clock::now();
            }

            else{
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

//A function to normalize the angle to -pi to pi
double constrainAngle(double x){
    x = fmod(x + 3.1415926, 6.2831852);
    if (x < 0)
        x += 6.2831852;
    return x - 3.1415926;
}

//A function that assigns scores to different ultrasound directions according to the angle between a ultrasound direction and the direction to the previous(current?) goal
double anglescores(double yaw, double x, double y){
  x = goal.target_pose.pose.position.x - pose.pose.position.x;
  y = goal.target_pose.pose.position.y - pose.pose.position.y;
  double gamma = atan2(y, x);
  // Calculate angle between direction of robot and the goal
  double angler2g = yaw - gamma + 1.57079632;//using 8 decimal places, if bug return to 7
  for(int i=0; i<8; i++){
    double ultraangle = constrainAngle(angler2g - 0.78539816*i); //Calculate normalized angle
    //return score for corresponding ultraangle
  }

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
