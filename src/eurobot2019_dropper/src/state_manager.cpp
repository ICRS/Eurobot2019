#include "state_manager.hpp"

StateManager::StateManager(ros::NodeHandle *nh) {
    eurobot2019_messages::drop_motors m;

    motor_targets_[IDLE] = m;
    motor_targets_[LOWER_STEPPER] = m;
    motor_targets_[RETRACT_PUSHER] = m;
    motor_targets_[EXTEND_PUSHER] = m;

    if(!nh->getParam("drop/idle_left_z",
                     motor_targets_[IDLE].left_z)) {
        ROS_ERROR("Failed to get param 'drop/idle_left_z'");
    }
    if(!nh->getParam("drop/idle_left_x",
                     motor_targets_[IDLE].left_x)) {
        ROS_ERROR("Failed to get param 'drop/idle_left_x'");
    }
    if(!nh->getParam("drop/idle_middle_z",
                     motor_targets_[IDLE].middle_z)) {
        ROS_ERROR("Failed to get param 'drop/idle_middle_z'");
    }
    if(!nh->getParam("drop/idle_middle_x",
                     motor_targets_[IDLE].middle_x)) {
        ROS_ERROR("Failed to get param 'drop/idle_middle_x'");
    }
    if(!nh->getParam("drop/idle_right_z",
                     motor_targets_[IDLE].right_z)) {
        ROS_ERROR("Failed to get param 'drop/idle_right_z'");
    }
    if(!nh->getParam("drop/idle_right_x",
                     motor_targets_[IDLE].right_x)) {
        ROS_ERROR("Failed to get param 'drop/idle_right_x'");
    }
    if(!nh->getParam("drop/retract_pusher_left_z",
                     motor_targets_[RETRACT_PUSHER].left_z)) {
        ROS_ERROR("Failed to get param 'drop/retract_pusher_left_z'");
    }
    if(!nh->getParam("drop/retract_pusher_left_x",
                     motor_targets_[RETRACT_PUSHER].left_x)) {
        ROS_ERROR("Failed to get param 'drop/retract_pusher_left_x'");
    }
    if(!nh->getParam("drop/retract_pusher_middle_z",
                     motor_targets_[RETRACT_PUSHER].middle_z)) {
        ROS_ERROR("Failed to get param 'drop/retract_pusher_middle_z'");
    }
    if(!nh->getParam("drop/retract_pusher_middle_x",
                     motor_targets_[RETRACT_PUSHER].middle_x)) {
        ROS_ERROR("Failed to get param 'drop/retract_pusher_middle_x'");
    }
    if(!nh->getParam("drop/retract_pusher_right_z",
                     motor_targets_[RETRACT_PUSHER].right_z)) {
        ROS_ERROR("Failed to get param 'drop/retract_pusher_right_z'");
    }
    if(!nh->getParam("drop/retract_pusher_right_x",
                     motor_targets_[RETRACT_PUSHER].right_x)) {
        ROS_ERROR("Failed to get param 'drop/retract_pusher_right_x'");
    }
    if(!nh->getParam("drop/lower_stepper_left_z",
                     motor_targets_[LOWER_STEPPER].left_z)) {
        ROS_ERROR("Failed to get param 'drop/lower_stepper_left_z'");
    }
    if(!nh->getParam("drop/lower_stepper_left_x",
                     motor_targets_[LOWER_STEPPER].left_x)) {
        ROS_ERROR("Failed to get param 'drop/lower_stepper_left_x'");
    }
    if(!nh->getParam("drop/lower_stepper_middle_z",
                     motor_targets_[LOWER_STEPPER].middle_z)) {
        ROS_ERROR("Failed to get param 'drop/lower_stepper_middle_z'");
    }
    if(!nh->getParam("drop/lower_stepper_middle_x",
                     motor_targets_[LOWER_STEPPER].middle_x)) {
        ROS_ERROR("Failed to get param 'drop/lower_stepper_middle_x'");
    }
    if(!nh->getParam("drop/lower_stepper_right_z",
                     motor_targets_[LOWER_STEPPER].right_z)) {
        ROS_ERROR("Failed to get param 'drop/lower_stepper_right_z'");
    }
    if(!nh->getParam("drop/lower_stepper_right_x",
                     motor_targets_[LOWER_STEPPER].right_x)) {
        ROS_ERROR("Failed to get param 'drop/lower_stepper_right_x'");
    }
    if(!nh->getParam("drop/extend_pusher_left_z",
                     motor_targets_[EXTEND_PUSHER].left_z)) {
        ROS_ERROR("Failed to get param 'drop/extend_pusher_left_z'");
    }
    if(!nh->getParam("drop/extend_pusher_left_x",
                     motor_targets_[EXTEND_PUSHER].left_x)) {
        ROS_ERROR("Failed to get param 'drop/extend_pusher_left_x'");
    }
    if(!nh->getParam("drop/extend_pusher_middle_z",
                     motor_targets_[EXTEND_PUSHER].middle_z)) {
        ROS_ERROR("Failed to get param 'drop/extend_pusher_middle_z'");
    }
    if(!nh->getParam("drop/extend_pusher_middle_x",
                     motor_targets_[EXTEND_PUSHER].middle_x)) {
        ROS_ERROR("Failed to get param 'drop/extend_pusher_middle_x'");
    }
    if(!nh->getParam("drop/extend_pusher_right_z",
                     motor_targets_[EXTEND_PUSHER].right_z)) {
        ROS_ERROR("Failed to get param 'drop/extend_pusher_right_z'");
    }
    if(!nh->getParam("drop/extend_pusher_right_x",
                     motor_targets_[EXTEND_PUSHER].right_x)) {
        ROS_ERROR("Failed to get param 'drop/extend_pusher_right_x'");
    }
