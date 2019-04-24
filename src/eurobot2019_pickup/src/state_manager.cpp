#include "state_manager.hpp"

StateManager::StateManager(ros::NodeHandle *nh) {
    eurobot2019_messages::grabber_motors m;

    motor_targets_[IDLE_OUT] = m;
    motor_targets_[IDLE_BACK] = m;
    motor_targets_[CLOSING] = m;
    motor_targets_[OPENING] = m;
    motor_targets_[VERTICAL_PUCK] = m;
    motor_targets_[ROTATING] = m;
    motor_targets_[DEPOSITING_GREEN] = m;
    motor_targets_[DEPOSITING_BLUE] = m;
    motor_targets_[DEPOSITING_RED] = m;

    if(!nh->getParam("pickup/idle_out_lift",
                     motor_targets_[IDLE_OUT].lift)) {
        ROS_ERROR("Failed to get param 'pickup/idle_out_lift'");
    }
    if(!nh->getParam("pickup/idle_out_twist",
                     motor_targets_[IDLE_OUT].twist)) {
        ROS_ERROR("Failed to get param 'pickup/idle_out_twist'");
    }

    if(!nh->getParam("pickup/idle_back_lift",
                     motor_targets_[IDLE_BACK].lift)) {
        ROS_ERROR("Failed to get param 'pickup/idle_back_lift'");
    }
    if(!nh->getParam("pickup/idle_back_twist",
                     motor_targets_[IDLE_BACK].twist)) {
        ROS_ERROR("Failed to get param 'pickup/idle_back_twist'");
    }

    if(!nh->getParam("pickup/vertical_lift",
                     motor_targets_[VERTICAL_PUCK].lift)) {
        ROS_ERROR("Failed to get param 'pickup/vertical_lift'");
    }
    if(!nh->getParam("pickup/idle_out_twist",
                     motor_targets_[VERTICAL_PUCK].twist)) {
        ROS_ERROR("Failed to get param 'pickup/idle_out_twist'");
    }

    if(!nh->getParam("pickup/open_close",
                     motor_targets_[CLOSING].open_close)) {
        ROS_ERROR("Failed to get param 'pickup/open_close'");
    }


    if(!nh->getParam("pickup/depositing_lift",
                     motor_targets_[DEPOSITING_GREEN].lift)) {
        ROS_ERROR("Failed to get param 'pickup/depositing_lift'");
    }
    if(!nh->getParam("pickup/depositing_green_twist",
                     motor_targets_[DEPOSITING_GREEN].twist)) {
        ROS_ERROR("Failed to get param 'pickup/depositing_green_twist'");
    }

    if(!nh->getParam("pickup/depositing_lift",
                     motor_targets_[DEPOSITING_BLUE].lift)) {
        ROS_ERROR("Failed to get param 'pickup/depositing_lift'");
    }
    if(!nh->getParam("pickup/depositing_blue_twist",
                     motor_targets_[DEPOSITING_BLUE].twist)) {
        ROS_ERROR("Failed to get param 'pickup/depositing_blue_twist'");
    }

    if(!nh->getParam("pickup/depositing_lift",
                     motor_targets_[DEPOSITING_RED].lift)) {
        ROS_ERROR("Failed to get param 'pickup/depositing_lift'");
    }
    if(!nh->getParam("pickup/depositing_red_twist",
                     motor_targets_[DEPOSITING_RED].twist)) {
        ROS_ERROR("Failed to get param 'pickup/depositing_red_twist'");
    }
}

eurobot2019_messages::grabber_motors
            StateManager::get_target(GrabberStates state) {
    switch(state) {
    case IDLE_OUT:
    case IDLE_BACK:
    case VERTICAL_PUCK:
        prev_state_.lift = motor_targets_[state].lift;
        prev_state_.twist = motor_targets_[state].twist;
        break;
    case CLOSING:
    case OPENING:
        prev_state_.open_close = motor_targets_[state].open_close;
        break;
    case ROTATING:
        prev_state_.puck_rotation = !prev_state_.puck_rotation;
        break;
    case DEPOSITING_GREEN:
    case DEPOSITING_BLUE:
    case DEPOSITING_RED:
        prev_state_.lift = motor_targets_[state].lift;
        prev_state_.twist = motor_targets_[state].twist;
        break;
    }
    return prev_state_;
}
