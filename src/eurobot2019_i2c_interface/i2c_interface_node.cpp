#define __PC_TEST__

#include <ros/ros.h>

#include "message_interface.hpp"
#include <string>
#include <vector>
#include <chrono>
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odom.h"
#include "i2c.hpp"
#define RADIUS 26.0 //from center of wheel to centre of roller, mm


// Preprocessor macro to check if we're running on RPi or test pc
#ifdef __PC_TEST__
#include <iostream> // For debugging
#else
#include <pigpio>
#endif

void send_drive_i2c_msg(I2C i2c_drive, const std::vector<float>& drive_motor_msg);
void get_drive_i2c_msg(I2C *i2c_drive, const std::vector<float>& drive_motor_msg);
std::vector<float> twist_to_wheel_vel(geometry_msgs::Twist cmd_vel);
void wheel_vel_to_odom(nav_msgs::Odom& current_pos, double& current_angle, const std::vector<float>& wheel_vel_msg);

struct Quaterniond{
    double w;
    double x;
    double y;
    double z;
};

int main(int argc, char **argv) {
    // Init ROS
    ros::init(argc, argv, "i2c_interface");

    // Get node handle
    ros::NodeHandle node_handle;

    std::chrono::high_resolution_clock::time_point t1 = high_resolution_clock::now();

    nav_msgs::Odom current_pos;
    double current_angle;

    node_handle.param("pose.point.x", current_pos.pose.pose.position.x, 0);
    node_handle.param("pose.point.y", current_pos.pose.pose.position.y, 0);
    node_handle.param("pose.point.z", current_pos.pose.pose.position.z, 0);
    node_handle.param("pose.orientation.quaternion.x", current_pos.pose.pose.orientation.x, 0);
    node_handle.param("pose.orientation.quaternion.y", current_pos.pose.pose.orientation.y, 0);
    node_handle.param("pose.orientation.quaternion.z", current_pos.pose.pose.orientation.z, 0);
    node_handle.param("pose.orientation.quaternion.w", current_pos.pose.pose.orientation.w, 0);
    node_handle.param("pose.orientation.angle", current_angle, 0);
    node_handle.param("twist.linear.x", current_pos.twist.twist.linear.x, 0);
    node_handle.param("twist.linear.y", current_pos.twist.twist.linear.y, 0);
    node_handle.param("twist.linear.z", current_pos.twist.twist.linear.z, 0);
    node_handle.param("twist.angular.x", current_pos.twist.twist.angular.x, 0);
    node_handle.param("twist.angular.y", current_pos.twist.twist.angular.y, 0);
    node_handle.param("twist.angular.z", current_pos.twist.twist.angular.z, 0);

    MessageInterface<std_msgs::Empty, std_msgs::Empty>
                drop_interface(100, "drop_status",
                               10 , "drop_motors");

    // Create Localisaion interface
    MessageInterface<std_msgs::Empty, std_msgs::Empty>
                grabber_interface(100, "grabber_status",
                                  10 , "grabber_motors");

    // check names of channels
    //publisher: nav_msgs/Odometry for Odom
    //subscriber: geometry_msgs/Twist for cmd_vel
    MessageInterface<std_msgs::Empty, std_msgs::Empty>
                navigation_interface(100, "odom",
                                     10 , "cmd_vel");



    // Control loop rate to be 100 Hz
    ros::Rate loop_rate(100);

    int count = 0;

    // initialise this to be zero for everything, (If using Empty messages to test ????)
    // std_msgs::Empty odometry_msg;

    // create i2c_drive to be used in send_drive_i2c_msg() to send along i2c_msg along I2C
    I2C i2c_drive;

    // Main loop
    while(ros::ok()) {
        // Do stuff
        // E.g. some calculations, create messages, read messages etc.

        // Read messages from relavent interfaces
        // messages to be sent to embed/arduino
        auto grabber_motors_msg = grabber_interface.get_msg();
        auto drop_motors_msg = drop_interface.get_msg();
        auto cmd_vel_msg = navigation_interface.get_msg();

        std::vector<float> drive_motor_msg = twist_to_wheel_vel(cmd_vel_msg);

        // Create a message and set the current message to be sent
        // This will continue to be sent until a new message is set
        // Get from I2C, convert from string to usable values
        std_msgs::Empty drop_status_msg;
        std_msgs::Empty grabber_status_msg;
        std::vector<float> wheel_vel_msg;
        std_msgs::Empty odometry_msg; //convert from wheel speeds to twist + add pose

        get_drive_i2c_msg(&i2c_drive, wheel_vel_msg);

        wheel_vel_to_odom(current_pos, current_angle, wheel_vel_msg);
        odometry_msg = current_pos;

        // Assume velocities are in a variable called drive_motor_msg

        send_drive_i2c_msg(drive_motor_msg);

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

std::vector<float> twist_to_wheel_vel(geometry_msgs::Twist cmd_vel){
  std::vector<float> drive_motor_msg;
  // Gives rotations per second (frequency)
  // x +ve is forward, y +ve is left, (need to check if same as navigation stack, if not change appropriately)
  drive_motor_msg.push_back((cmd_vel.linear.x - cmd_vel.linear.y - 261.236363668644*cmd_vel.angular.z)/RADIUS); //top left
  drive_motor_msg.push_back((cmd_vel.linear.x + cmd_vel.linear.y + 261.236363668644*cmd_vel.angular.z)/RADIUS); //top right
  drive_motor_msg.push_back((cmd_vel.linear.x + cmd_vel.linear.y - 187.763636356656*cmd_vel.angular.z)/RADIUS); //bottom left
  drive_motor_msg.push_back((cmd_vel.linear.x - cmd_vel.linear.y + 187.763636356656*cmd_vel.angular.z)/RADIUS); //bottom right
}

//converts wheel angular vels to the nav_msgs/Odometry format by updating current_pos.
//current_angle is the 'euler' yaw, is saved for easy access
void wheel_vel_to_odom(nav_msgs::Odom& current_pos, double& current_angle, const std::vector<float>& wheel_vel_msg){
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::chrono::high_resolution_clock::time_point t1 = high_resolution_clock::now();

    double linear_x = (wheel_vel_msg[0] + wheel_vel_msg[1] + wheel_vel_msg[2] + wheel_vel_msg[3])*(RADIUS/4.0);
    double linear_y = (-wheel_vel_msg[0] + wheel_vel_msg[1] + wheel_vel_msg[2] - wheel_vel_msg[3])*(RADIUS/4.0);
    double angular_z = (-0.003827951*wheel_vel_msg[0] + 0.003827951*wheel_vel_msg[1] - 0.005325845*wheel_vel_msg[2] + 0.005325845*wheel_vel_msg[3])*(RADIUS/4.0);

    double dt = time_span.count();
    double delta_x = (linear_x * cos(current_angle) - linear_y * sin(current_angle)) * dt;
    double delta_y = (linear_x * sin(current_angle) + linear_y * cos(current_angle)) * dt;
    double delta_th = angular_z * dt;

    Quaterniond q = toQuaternion(angular_z, 0, 0);

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




void send_drive_i2c_msg(I2C *i2c_drive, const std::vector<float>& drive_motor_msg){
  std::string drive_i2c_msg;
  // save pointer for speed of  in char pointer d
  char *d = &drive_motor_msg[0];
  for(int i = 0; i < 4; i++){
    // dereference every byte of M1 as a char into i2c_msg
    drive_i2c_msg += *(d++);
  }

  char *d = &drive_motor_msg[1];
  for(int i = 0; i < 4; i++){
    drive_i2c_msg += *(d++);
  }

  char *d = &drive_motor_msg[2];
  for(int i = 0; i < 4; i++){
    drive_i2c_msg += *(d++);
  }

  char *d = &drive_motor_msg[3];
  for(int i = 0; i < 4; i++){
    drive_i2c_msg += *(d++);
  }
  //Send the converted drive_motor_msg from the interface to a target unknown in DRIVE.
  i2c_drive->write(DRIVE, drive_i2c_msg);

  //i2c_msg is now a string containing characters encoding velocity values (float32)
  // for drive motors in order top left, top right, bottom left, bottom right

  // send i2c_msg along I2C
}

void get_drive_i2c_msg(I2C *i2c_drive, std::vector<float>& wheel_vel_msg){
    std::string wheel_vel_i2c_msg;
    wheel_vel_i2c_msg = i2c_drive->read(DRIVE, 16); //reads motor values, length assumed to be 16
    char str[16];
    strcpy(str, "%s", wheel_vel_i2c_msg.c_str());
    float* tmp = &str;
    
    for(int i = 0; i < 4; i++){
        wheel_vel_msg.push_back(*(tmp++));
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
    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;
    return q;
}
