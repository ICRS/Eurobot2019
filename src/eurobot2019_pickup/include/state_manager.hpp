#ifndef __STATE_MANAGER__
#define __STATE_MANAGER__

#include <ros/ros.h>
#include <eurobot2019_messages/grabber_motors.h>
#include <map>

#include "message_interface.hpp"

enum GrabberStates {
    IDLE_OUT,
    IDLE_BACK,
    CLOSING,
    OPENING,
    VERTICAL_PUCK,
    ROTATING,
    DEPOSITING_GREEN,
    DEPOSITING_BLUE,
    DEPOSITING_RED,
    BACK_UP
};

class StateManager {
public:
    StateManager(ros::NodeHandle *nh);

    eurobot2019_messages::grabber_motors get_target(GrabberStates state);
private:
    std::map<GrabberStates, eurobot2019_messages::grabber_motors> motor_targets_;
    eurobot2019_messages::grabber_motors prev_state_;
};

#endif // __STATE_MANAGER__
