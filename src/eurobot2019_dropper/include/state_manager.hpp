#ifndef __STATE_MANAGER__
#define __STATE_MANAGER__

#include <ros/ros.h>
#include <eurobot2019_messages/drop_motors.h>
#include <eurobot2019_messages/drop_status.h>
#include <map>

#include "message_interface.hpp"

enum DropState {
    IDLE,
    LOWER_STEPPER,
    RETRACT_PUSHER,
    EXTEND_PUSHER
};

class StateManager {
public:
    StateManager(ros::NodeHandle *nh);

    eurobot2019_messages::drop_motors get_target(DropState state);
private:
    std::map<DropState, eurobot2019_messages::drop_motors> motor_targets_;
    eurobot2019_messages::drop_motors prev_state_;
};

#endif // __STATE_MANAGER__
