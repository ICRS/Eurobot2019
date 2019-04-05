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

    if(!nh->getParam("pickup/idle_out_z_pos",
                     motor_targets_[IDLE_OUT].z_pos_mm)) {
        ROS_ERROR("Failed to get param 'pickup/idle_out_z_pos'");
    }
    if(!nh->getParam("pickup/idle_out_z_twist",
                     motor_targets_[IDLE_OUT].z_twist_rad)) {
        ROS_ERROR("Failed to get param 'pickup/idle_out_z_twist'");
    }

    if(!nh->getParam("pickup/idle_back_z_pos",
                     motor_targets_[IDLE_BACK].z_pos_mm)) {
        ROS_ERROR("Failed to get param 'pickup/idle_back_z_pos'");
    }
    if(!nh->getParam("pickup/idle_back_z_twist",
                     motor_targets_[IDLE_BACK].z_twist_rad)) {
        ROS_ERROR("Failed to get param 'pickup/idle_back_z_twist'");
    }

    if(!nh->getParam("pickup/vertical_z_pos",
                     motor_targets_[VERTICAL_PUCK].z_pos_mm)) {
        ROS_ERROR("Failed to get param 'pickup/vertical_z_pos'");
    }
    if(!nh->getParam("pickup/idle_out_z_twist",
                     motor_targets_[VERTICAL_PUCK].z_twist_rad)) {
        ROS_ERROR("Failed to get param 'pickup/idle_out_z_twist'");
    }

    if(!nh->getParam("pickup/close_pos",
                     motor_targets_[CLOSING].open_pos_mm)) {
        ROS_ERROR("Failed to get param 'pickup/close_pos'");
    }

    if(!nh->getParam("pickup/open_pos",
                     motor_targets_[OPENING].open_pos_mm)) {
        ROS_ERROR("Failed to get param 'pickup/open_pos'");
    }
    
    if(!nh->getParam("pickup/depositing_z_pos",
                     motor_targets_[DEPOSITING_GREEN].z_pos_mm)) {
        ROS_ERROR("Failed to get param 'pickup/depositing_z_pos'");
    }
    if(!nh->getParam("pickup/depositing_green_z_twist",
                     motor_targets_[DEPOSITING_GREEN].z_twist_rad)) {
        ROS_ERROR("Failed to get param 'pickup/depositing_green_z_twist'");
    }

    if(!nh->getParam("pickup/depositing_z_pos",
                     motor_targets_[DEPOSITING_BLUE].z_pos_mm)) {
        ROS_ERROR("Failed to get param 'pickup/depositing_z_pos'");
    }
    if(!nh->getParam("pickup/depositing_blue_z_twist",
                     motor_targets_[DEPOSITING_BLUE].z_twist_rad)) {
        ROS_ERROR("Failed to get param 'pickup/depositing_blue_z_twist'");
    }

    if(!nh->getParam("pickup/depositing_z_pos",
                     motor_targets_[DEPOSITING_RED].z_pos_mm)) {
        ROS_ERROR("Failed to get param 'pickup/depositing_z_pos'");
    }
    if(!nh->getParam("pickup/depositing_red_z_twist",
                     motor_targets_[DEPOSITING_RED].z_twist_rad)) {
        ROS_ERROR("Failed to get param 'pickup/depositing_red_z_twist'");
    }
}

eurobot2019_messages::grabber_motors 
            StateManager::get_target(GrabberStates state) {
    switch(state) {
    case IDLE_OUT:
    case IDLE_BACK:
    case VERTICAL_PUCK:
        prev_state_.z_pos_mm = motor_targets_[state].z_pos_mm;
        prev_state_.z_twist_rad = motor_targets_[state].z_twist_rad;
        break;
    case CLOSING:
    case OPENING:
        prev_state_.open_pos_mm = motor_targets_[state].open_pos_mm;
        break;
    case ROTATING:
        prev_state_.servo_state = !prev_state_.servo_state;
        break;
    case DEPOSITING_GREEN:
    case DEPOSITING_BLUE:
    case DEPOSITING_RED:
        prev_state_.z_pos_mm = motor_targets_[state].z_pos_mm;
        prev_state_.z_twist_rad = motor_targets_[state].z_twist_rad;
        break;
    }
    return prev_state_;
}
