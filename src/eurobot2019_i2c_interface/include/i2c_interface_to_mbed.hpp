#ifndef __I2C_INTERFACE_TO_MBED_HPP__
#define __I2C_INTERFACE_TO_MBED_HPP__

#include <string>
#include <vector>
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "eurobot2019_messages/drop_motors.h"
#include "eurobot2019_messages/grabber_motors.h"
#include <i2c.hpp>

#include <ros/ros.h>

#ifndef __arm__
#include <iostream> // for debugging
#else
#include <pigpio.h>
#endif // __arm__

#define RADIUS 0.026 //from center of wheel to centre of roller, m

//I2C Class creation?
class I2C_interface_to_mbed {
public:
    I2C_interface_to_mbed();
        ~I2C_interface_to_mbed();

        void send_drive_i2c_msg(const geometry_msgs::Twist& cmd_vel_msg);

        void get_drive_i2c_msg(std::vector<float>& wheel_vel_msg);

        void send_grabber_i2c_msg(const eurobot2019_messages::grabber_motors& grabber_motors_msg);

        void get_grabber_i2c_msg(eurobot2019_messages::grabber_motors& grabber_status_msg);

        void send_dropper_i2c_msg(const eurobot2019_messages::drop_motors& drop_motors_msg);

        void get_dropper_i2c_msg(eurobot2019_messages::drop_motors& drop_status_msg);

private:
    // create i2c_handler to be used in handling communication along i2c
    I2C i2c_handler_;
};

#endif // __I2C_INTERFACE_TO_MBED_HPP__
