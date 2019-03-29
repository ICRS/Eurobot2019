#define __PC_TEST__

#include <ros/ros.h>

#include "message_interface.hpp"
#include <string>
#include <vector>
#include <chrono>
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "eurobot2019_messages/drop_status.h"
#include "eurobot2019_messages/drop_motors.h"
#include "eurobot2019_messages/grabber_motors.h"
#include <i2c.hpp>
#include <i2c_interface_to_mbed.hpp>
#define RADIUS 26.0 //from center of wheel to centre of roller, mm


// Preprocessor macro to check if we're running on RPi or test pc
#ifdef __PC_TEST__
#include <iostream> // For debugging
#else
#include <pigpio>
#endif

struct Quaterniond{
    double w;
    double x;
    double y;
    double z;
};

void wheel_vel_to_odom(nav_msgs::Odometry& current_pos, double& current_angle, const std::vector<float>& wheel_vel_msg);
Quaterniond toQuaternion(double yaw, double pitch, double roll);

std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
// This time is set as a global variable to allow first run of wheel_vel_to_odom() without passing t1 into function


int main(int argc, char **argv) {
    // Init ROS
    ros::init(argc, argv, "i2c_interface");

    // Get node handle
    ros::NodeHandle node_handle;

    nav_msgs::Odometry current_pos;
    double current_angle;

    I2C_interface_to_mbed i2c_interface;

    /*
    Instead, do like this:  if(!node_handle.getParam('pose.point.x', double){
        ROS ERROR("Failed to get ");
    }
    */

    // Can make this a param?
    for(int i = 0; i < 36; i++){
        current_pos.pose.covariance[i] = 0;
    }

    for(int i = 0; i < 36; i++){
        current_pos.twist.covariance[i] = 0;
    }

    node_handle.param<double>("pose_point_x", current_pos.pose.pose.position.x, 0);
    node_handle.param<double>("pose_point_y", current_pos.pose.pose.position.y, 0);
    node_handle.param<double>("pose_point_z", current_pos.pose.pose.position.z, 0);
    node_handle.param<double>("pose_orientation_quaternion_x", current_pos.pose.pose.orientation.x, 0);
    node_handle.param<double>("pose_orientation_quaternion_y", current_pos.pose.pose.orientation.y, 0);
    node_handle.param<double>("pose_orientation_quaternion_z", current_pos.pose.pose.orientation.z, 0);
    node_handle.param<double>("pose_orientation_quaternion_w", current_pos.pose.pose.orientation.w, 0);
    node_handle.param<double>("pose_orientation_angle", current_angle, 0);
    node_handle.param<double>("twist_linear_x", current_pos.twist.twist.linear.x, 0);
    node_handle.param<double>("twist_linear_y", current_pos.twist.twist.linear.y, 0);
    node_handle.param<double>("twist_linear_z", current_pos.twist.twist.linear.z, 0);
    node_handle.param<double>("twist_angular_x", current_pos.twist.twist.angular.x, 0);
    node_handle.param<double>("twist_angular_y", current_pos.twist.twist.angular.y, 0);
    node_handle.param<double>("twist_angular_z", current_pos.twist.twist.angular.z, 0);

    MessageInterface<eurobot2019_messages::drop_status, eurobot2019_messages::drop_motors>
                drop_interface(100, "drop_status",
                               10 , "drop_motors");

    // Create Localisaion interface
    MessageInterface<std_msgs::Empty, eurobot2019_messages::grabber_motors>
                grabber_interface(100, "grabber_status",
                                  10 , "grabber_motors");

    // check names of channels
    //publisher: nav_msgs/Odometry for Odom
    //subscriber: geometry_msgs/Twist for cmd_vel
    MessageInterface<nav_msgs::Odometry, geometry_msgs::Twist>
                navigation_interface(100, "odom",
                                     10 , "cmd_vel");



    // Control loop rate to be 100 Hz
    ros::Rate loop_rate(100);

    int count = 0;

    // initialise this to be zero for everything, (If using Empty messages to test ????)
    // std_msgs::Empty odometry_msg;

    // Main loop
    while(ros::ok()) {
        // Do stuff
        // E.g. some calculations, create messages, read messages etc.

        // Read messages from relevant interfaces
        // messages to be sent to embed/arduino
        auto grabber_motors_msg = grabber_interface.get_msg();
        auto drop_motors_msg = drop_interface.get_msg();
        auto cmd_vel_msg = navigation_interface.get_msg();

        i2c_interface.send_grabber_i2c_msg(grabber_motors_msg);
        i2c_interface.send_dropper_i2c_msg(drop_motors_msg);
        i2c_interface.send_drive_i2c_msg(cmd_vel_msg);

        // Create a message and set the current message to be sent
        // This will continue to be sent until a new message is set
        // Get from I2C, convert from string to usable values
        eurobot2019_messages::drop_status drop_status_msg;
        std_msgs::Empty grabber_status_msg; //should be eurobot2019_messages::grabber_status
        std::vector<float> wheel_vel_msg;
        nav_msgs::Odometry odometry_msg; //convert from wheel speeds to twist + add pose

        i2c_interface.get_dropper_i2c_msg(drop_status_msg);
        i2c_interface.get_drive_i2c_msg(wheel_vel_msg);
        wheel_vel_to_odom(current_pos, current_angle, wheel_vel_msg);
        odometry_msg = current_pos;

        // Assume velocities are in a variable called drive_motor_msg

        // messages taken from embed/arduino to be sent to nodes
        drop_interface.set_msg(drop_status_msg);
        grabber_interface.set_msg(grabber_status_msg);
        navigation_interface.set_msg(odometry_msg);

/* EXAMPLE of ifdef
#ifdef __PC_TEST__
        std::cout << "Set pin 14 to 1" << std::endl;
#else
        gpioWrite(14, 1);
#endif
*/

        // Keep update frequency
        loop_rate.sleep();
    }
}

//Assume function that turns cmd_vel into speeds for every motor, this is twist_to_wheel_vel()
// cmd_vel_msg ---> drive_msg, see holonomic_vel_kinematic
// driver_motor_msg contains motor velocities in this order:
// 0 is top left, 1 is top right, 2 is bottom left, 3 is botom right:
/*  float32 0
    float32 1
    float32 2
    float32 3
*/

//converts wheel angular vels to the nav_msgs/Odometry format by updating current_pos.
//current_angle is the 'euler' yaw, is saved for easy access
void wheel_vel_to_odom(nav_msgs::Odometry& current_pos, double& current_angle, const std::vector<float>& wheel_vel_msg){
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    double linear_x = (wheel_vel_msg[0] + wheel_vel_msg[1] + wheel_vel_msg[2] + wheel_vel_msg[3])*(RADIUS/4.0);
    double linear_y = (-wheel_vel_msg[0] + wheel_vel_msg[1] + wheel_vel_msg[2] - wheel_vel_msg[3])*(RADIUS/4.0);
    double angular_z = (-0.003827951*wheel_vel_msg[0] + 0.003827951*wheel_vel_msg[1] - 0.005325845*wheel_vel_msg[2] + 0.005325845*wheel_vel_msg[3])*(RADIUS/4.0);

    double dt = time_span.count();
    double delta_x = (linear_x * cos(current_angle) - linear_y * sin(current_angle)) * dt;
    double delta_y = (linear_x * sin(current_angle) + linear_y * cos(current_angle)) * dt;
    double delta_th = angular_z * dt;

    Quaterniond q;
    q = toQuaternion(angular_z, 0, 0);

    current_angle += delta_th;
    current_pos.pose.pose.position.x += delta_x;
    current_pos.pose.pose.position.y += delta_y;
    current_pos.twist.twist.linear.x = linear_x;
    current_pos.twist.twist.linear.y = linear_y;
    current_pos.twist.twist.angular.z = angular_z;
    current_pos.pose.pose.orientation.x = q.x;
    current_pos.pose.pose.orientation.y = q.y;
    current_pos.pose.pose.orientation.z = q.z;
    current_pos.pose.pose.orientation.w = q.w;
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
