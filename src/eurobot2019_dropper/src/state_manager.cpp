#include "state_manager.hpp"

StateManager::StateManager(ros::NodeHandle *nh) {
    eurobot2019_messages::drop_motors m;

    motor_targets_[IDLE_L] = m;
    motor_targets_[LOWER_STEPPER_L] = m;
    motor_targets_[RETRACT_PUSHER_L] = m;
    motor_targets_[EXTEND_PUSHER_L] = m;
    motor_targets_[IDLE_R] = m;
    motor_targets_[LOWER_STEPPER_R] = m;
    motor_targets_[RETRACT_PUSHER_R] = m;
    motor_targets_[EXTEND_PUSHER_R] = m;
    motor_targets_[IDLE_M] = m;
    motor_targets_[LOWER_STEPPER_M] = m;
    motor_targets_[RETRACT_PUSHER_M] = m;
    motor_targets_[EXTEND_PUSHER_M] = m;
std::cout << "set up map" << std::endl;
    if(!nh->getParam("drop/idle_x",
                     motor_targets_[IDLE_L].left_x)) {
        ROS_ERROR("Failed to get param 'drop/idle_left_x'");
    }
    if(!nh->getParam("drop/idle_x",
                     motor_targets_[IDLE_M].middle_x)) {
        ROS_ERROR("Failed to get param 'drop/idle_middle_x'");
    }
    if(!nh->getParam("drop/idle_x",
                     motor_targets_[IDLE_R].right_x)) {
        ROS_ERROR("Failed to get param 'drop/idle_right_x'");
    }
    if(!nh->getParam("drop/retract_pusher_x",
                     motor_targets_[RETRACT_PUSHER_L].left_x)) {
        ROS_ERROR("Failed to get param 'drop/retract_pusher_left_x'");
    }
    if(!nh->getParam("drop/retract_pusher_x",
                     motor_targets_[RETRACT_PUSHER_M].middle_x)) {
        ROS_ERROR("Failed to get param 'drop/retract_pusher_middle_x'");
    }
    if(!nh->getParam("drop/retract_pusher_x",
                     motor_targets_[RETRACT_PUSHER_R].right_x)) {
        ROS_ERROR("Failed to get param 'drop/retract_pusher_right_x'");
    }
    if(!nh->getParam("drop/lower_stepper_z",
                     motor_targets_[LOWER_STEPPER_L].left_z)) {
        ROS_ERROR("Failed to get param 'drop/lower_stepper_left_z'");
    }
    if(!nh->getParam("drop/lower_stepper_z",
                     motor_targets_[LOWER_STEPPER_M].middle_z)) {
        ROS_ERROR("Failed to get param 'drop/lower_stepper_middle_z'");
    }
    if(!nh->getParam("drop/lower_stepper_z",
                     motor_targets_[LOWER_STEPPER_R].right_z)) {
        ROS_ERROR("Failed to get param 'drop/lower_stepper_right_z'");
    }
    if(!nh->getParam("drop/extend_pusher_x",
                     motor_targets_[EXTEND_PUSHER_L].left_x)) {
        ROS_ERROR("Failed to get param 'drop/extend_pusher_left_x'");
    }
    if(!nh->getParam("drop/extend_pusher_x",
                     motor_targets_[EXTEND_PUSHER_M].middle_x)) {
        ROS_ERROR("Failed to get param 'drop/extend_pusher_middle_x'");
    }
    if(!nh->getParam("drop/extend_pusher_x",
                     motor_targets_[EXTEND_PUSHER_R].right_x)) {
        ROS_ERROR("Failed to get param 'drop/extend_pusher_right_x'");
    }

    std::cout << "got all params" << std::endl;
}

eurobot2019_messages::drop_motors
            StateManager::get_target(DropState state) {
    switch(state) {
    case IDLE_L:
    case RETRACT_PUSHER_L:
    case EXTEND_PUSHER_L:
        prev_state_.left_x = motor_targets_[state].left_x;
        break;
    case LOWER_STEPPER_L:
        prev_state_.left_z = motor_targets_[state].left_z;
        break;

    case IDLE_R:
    case RETRACT_PUSHER_R:
    case EXTEND_PUSHER_R:
        prev_state_.right_x = motor_targets_[state].right_x;
        break;
    case LOWER_STEPPER_R:
        prev_state_.right_z = motor_targets_[state].right_z;
        break;

    case IDLE_M:
    case RETRACT_PUSHER_M:
    case EXTEND_PUSHER_M:
        prev_state_.middle_x = motor_targets_[state].middle_x;
        break;
    case LOWER_STEPPER_M:
        prev_state_.middle_z = motor_targets_[state].middle_z;
        break;
    }
    return prev_state_;
}
