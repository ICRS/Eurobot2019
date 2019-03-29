#define __PC_TEST__

#include <ros/ros.h>

#include "message_interface.hpp"
#include <string>
#include <vector>
#include <i2c.hpp>

// Preprocessor macro to check if we're running on RPi or test pc
#ifdef __PC_TEST__
#include <iostream> // For debugging
#else
#include <pigpio>
#endif

void send_drive_i2c_msg(I2C i2c_drive, const std::vector<float32>& drive_motor_msg);
void Twist_to_wheel_vel(geometry_msgs::Twist cmd_vel);
void Wheel_vel_to_Odom(std::vector<float32> drive_motor_msg, std::vector<float32> drive_motor_msg);

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
    //publisher: geometry_msgs/Twist for cmd_vel
    MessageInterface<std_msgs::Empty, std_msgs::Empty>
                navigation_interface(100, "odom",
                                     10 , "cmd_vel");



    // Control loop rate to be 100 Hz
    ros::Rate loop_rate(100);

    int count = 0;

    // initialise this to be zero for everything
    std_msgs::Empty odometry_msg;

    // create i2c_drive to be used in send_drive_i2c_msg() to send along i2c_msg along I2C
    I2C i2c_drive;

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

        // Assume velocities are in a variable called drive_motor_msg

        send_drive_i2c_msg(&i2c_drive, drive_motor_msg);

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

std::vector<float32> Twist_to_wheel_vel(geometry_msgs::Twist cmd_vel){
  std::vector<float32> drive_motor_msg;
  // Gives rotations per second (frequency)
  // x +ve is forward, y +ve is left
  drive_motor_msg[0] = (cmd_vel.linear.x - cmd_vel.linear.y - 261.236363668644*cmd_vel.angular.z)/26; //top left
  drive_motor_msg[1] = (cmd_vel.linear.x + cmd_vel.linear.y + 261.236363668644*cmd_vel.angular.z)/26; //top right
  drive_motor_msg[2] = (cmd_vel.linear.x + cmd_vel.linear.y - 187.763636356656*cmd_vel.angular.z)/26; //bottom left
  drive_motor_msg[3] = (cmd_vel.linear.x - cmd_vel.linear.y + 187.763636356656*cmd_vel.angular.z)/26; //bottom right
}

void Wheel_vel_to_Odom(/*Way to access current position */, const std::vector<float32>& drive_motor_msg){
  Odom.Twist.linear.x = (drive_motor_msg[0] + drive_motor_msg[1] + drive_motor_msg[2] + drive_motor_msg[3])*6.5;
  Odom.Twist.linear.y = (-drive_motor_msg[0] + drive_motor_msg[1] + drive_motor_msg[2] - drive_motor_msg[3])*6.5;
  Odom.Twist.angular.z = (-0.003827951*drive_motor_msg[0] + 0.003827951*drive_motor_msg[1] - 0.005325845*drive_motor_msg[2] + 0.005325845*drive_motor_msg[3])*6.5;

  double dt = // ??, std::chrono;
  double delta_x = (Odom.Twist.linear.x * cos(Odom.Twist.angular.z) - Odom.Twist.linear.y * sin(Odom.Twist.angular.z)) * dt;
  double delta_y = (Odom.Twist.linear.x * sin(Odom.Twist.angular.z) + Odom.Twist.linear.y * cos(Odom.Twist.angular.z)) * dt;
  double delta_th = /*current angle*/ * dt;


}




void send_drive_i2c_msg(I2C *i2c_drive, const std::vector<float32>& drive_motor_msg){
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

}
