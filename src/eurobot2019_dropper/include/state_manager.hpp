#ifndef __STATE_MANAGER__
#define __STATE_MANAGER__

#include <ros/ros.h>
#include <eurobot2019_messages/drop_motors.h>
#include <eurobot2019_messages/drop_command.h>
#include <map>

#include "message_interface.hpp"

enum DropState {
    IDLE_L,
    LOWER_STEPPER_L,
    RETRACT_PUSHER_L,
    EXTEND_PUSHER_L,
    IDLE_R,
    LOWER_STEPPER_R,
    RETRACT_PUSHER_R,
    EXTEND_PUSHER_R,
    IDLE_M,
    LOWER_STEPPER_M,
    RETRACT_PUSHER_M,
    EXTEND_PUSHER_M
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
