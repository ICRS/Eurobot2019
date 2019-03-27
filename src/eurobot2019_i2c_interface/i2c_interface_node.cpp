#define __PC_TEST__

#include <ros/ros.h>

#include "message_interface.hpp"
#include <string>

// Preprocessor macro to check if we're running on RPi or test pc
#ifdef __PC_TEST__
#include <iostream> // For debugging
#else
#include <pigpio>
#endif

std::string get_drive_i2c_msg(std_msgs::Empty);

int main(int argc, char **argv) {
    // Init ROS
    ros::init(argc, argv, "i2c_interface");

    // Get node handle
    ros::NodeHandle node_handle;

    MessageInterface<std_msgs::Empty, std_msgs::Empty>
                drop_interface(100, "drop_status",
                               10 , "drop_motors");

    // Create Localisaion interface
    MessageInterface<std_msgs::Empty, std_msgs::Empty>
                grabber_interface(100, "grabber_status",
                                  10 , "grabber_motors");

    // check names of channels
    //publisher: nav_msgs/Odometry for Odom
    //publisher: geomrtry_msgs/Twist for cmd_vel
    MessageInterface<std_msgs::Empty, std_msgs::Empty>
                navigation_interface(100, "odom",
                                     10 , "cmd_vel");


    // Control loop rate to be 100 Hz
    ros::Rate loop_rate(100);

    int count = 0;

    // Main loop
    while(ros::ok()) {
        // Do stuff
        // E.g. some calculations, create messages, read messages etc.

        // Read messages from relavent interfaces
        auto grabber_motors_msg = grabber_interface.get_msg();
        auto drop_motors_msg = drop_interface.get_msg();
        auto cmd_vel_msg = navigation_interface.get_msg();

        // Create a message and set the current message t be sent
        // This will continue to be sent until a new message is set
        // Get from I2C(?)
        std_msgs::Empty drop_status_msg;
        std_msgs::Empty grabber_status_msg;
        std_msgs::Empty odometry_msg;

        // Assume velocities are in a variable called drive_msg

        send_drive_i2c_msg(drive_msg);


        drop_interface.set_msg(drop_status_msg);
        grabber_interface.set_msg(grabber_status_msg);
        navigation_interface.set_msg(odometry_msg);

#ifdef __PC_TEST__
        std::cout << "Set pin 14 to 1" << std::endl;
#else
        gpioWrite(14, 1);
#endif

        // Keep update frequency
        loop_rate.sleep();
    }
}

//Assume function that turns cmd_vel into speeds for every motor
// cmd_vel_msg ---> drive_msg, see holonomic_vel_kinematic
// In this order:
//From top left, clock-wise
// M1 is top left, M2 is top right, M3 is bottom right, M4 is botom left:
/*  float32 M1
    float32 M2
    float32 M3
    float32 M4
*/


void send_drive_i2c_msg(drive_motor_msg){
  std::string i2c_msg;
  // save pointer for speed of M1 in char pointer d
  char *d = &drive_motor_msg.M1;
  for(int i = 0; i < 4; i++){
    // dereference every byte of M1 as a char into i2c_msg
    i2c_msg += *(d++);
  }

  char *d = &drive_motor_msg.M2;
  for(int i = 0; i < 4; i++){
    i2c_msg += *(d++);
  }

  char *d = &drive_motor_msg.M3;
  for(int i = 0; i < 4; i++){
    i2c_msg += *(d++);
  }

  char *d = &drive_motor_msg.M4;
  for(int i = 0; i < 4; i++){
    i2c_msg += *(d++);
  }

  // send along i2c_msg along I2C
}
