#include <ros/ros.h>
#include <eurobot2019_messages/grabber_motors.h>
#include <eurobot2019_messages/pickup.h>
#include <queue>

#include <std_msgs/Int32.h>

#include "message_interface.hpp"
#include "state_manager.hpp"

// Function to check if floating point numbers are close enough
bool approx_equal(float a, float b) {
    return fabs(a-b) < 1e-2;
}

int main(int argc, char **argv) {
    // Initialise ROS
    ros::init(argc, argv, "pickup");

    // Get node handle
    ros::NodeHandle nh;

    // Create interface with the hardware node
    MessageInterface<eurobot2019_messages::grabber_motors,
                     eurobot2019_messages::grabber_motors>
                hardware(10, "grabber_motors", 10, "grabber_status");

    // Create interface to the tactics node
    MessageInterface<std_msgs::Int32, eurobot2019_messages::pickup>
                command(10, "pickup_status", 10, "pickup");

    // Create state manager instance
    StateManager state_manager(&nh);

    // Message to be published, and the final target
    eurobot2019_messages::grabber_motors motor_msg, target_msg;

    // Set the target message for the grabber
    hardware.set_msg(motor_msg);

    // Create the old message
    eurobot2019_messages::pickup old_command_msg;

    // Create the target message for tactics node
    std_msgs::Int32 pickup_status_msg;

    // Create a queue to manage the state order
    std::queue<GrabberStates> state_queue;
    state_queue.push(IDLE_BACK);

    // Stack is size 1 so set the msg
    pickup_status_msg.data = 1;
    command.set_msg(pickup_status_msg);


    // 100 Hz update rate
    ros::Rate sleeper(100);

    while(ros::ok()) {
        // Get the message from command
        auto command_msg = command.get_msg();

        // Do stuff with the message if it has changed
        if(command_msg.is_vertical != old_command_msg.is_vertical ||
           command_msg.pos_reached != old_command_msg.pos_reached ||
           command_msg.colour != old_command_msg.colour) {

            ROS_INFO("New command message received");
            old_command_msg = command_msg;

            // Idle command means don't pick up a puck at the moment
            if(command_msg.colour == 0) {
                state_queue.push(OPENING);
                state_queue.push(IDLE_OUT);
            }
            else if(command_msg.colour == 5) {
                state_queue.push(OPENING);
                state_queue.push(IDLE_BACK);
            }
            else {
                if(command_msg.is_vertical) {
                    // Move to the correct z position
                    state_queue.push(VERTICAL_PUCK);
                    state_queue.push(OPENING);
                }
                else {
                    // Move to the correct z position
                    state_queue.push(IDLE_OUT);
                    state_queue.push(OPENING);
                }
                if(command_msg.pos_reached) {
                    /* We've already added the correct z pos, we now:
                     *
                     * 1. Close grabber
                     * 2. Rotate if necessary
                     * 3. Move to colour location
                     * 4. Release the puck
                     */
                    state_queue.push(CLOSING);
                    if(command_msg.is_vertical)
                        state_queue.push(ROTATING);
                    switch(command_msg.colour) {
                    case 1:
                        state_queue.push(DEPOSITING_RED);
                        state_queue.push(OPENING);
                        break;
                    case 2:
                        state_queue.push(DEPOSITING_GREEN);
                        state_queue.push(OPENING);
                        break;
                    case 3:
                        state_queue.push(DEPOSITING_BLUE);
                        state_queue.push(OPENING);
                        break;
                    case 4:
                        state_queue.push(IDLE_OUT);
                        break;
                    }
                    // Un-rotate at the end
                    if(command_msg.is_vertical)
                        state_queue.push(ROTATING);
                }
            }
        }

        // Update if we've arrived at the required position
        auto grabber_pos = hardware.get_msg();
        // If the current twist is non middle
        // And current lift is not at the target, set twist to middle
        if((grabber_pos.twist != 1) &&
           target_msg.lift != grabber_pos.lift) {
            motor_msg.twist = 1;
        }
        // If target lift is not equal to current one and twist is 1, move lift
        else if(target_msg.lift != grabber_pos.lift) {
            motor_msg.lift = target_msg.lift;
        }
        // If lift is correct but twist is not, move twist
        else if(target_msg.twist != grabber_pos.twist) {
            motor_msg.twist = target_msg.twist;
        }
        // If lift and twist are in the correct position, adjust open/close
        else if(target_msg.open_close !=  grabber_pos.open_close) {
            motor_msg.open_close  = target_msg.open_close;
        }
        // Finally if the puck_rotation is wrong, update that
        else if(target_msg.puck_rotation != grabber_pos.puck_rotation) {
            motor_msg.puck_rotation = target_msg.puck_rotation;
        }
        // Then everything is in the correct place, so update the state
        else {
            // Update message before the queue size changes
            // so it include the currently processing one
            pickup_status_msg.data = state_queue.size();
            if(state_queue.size()) {
                target_msg = state_manager.get_target(state_queue.front());
                state_queue.pop();
            }
        }

        hardware.set_msg(motor_msg);

        // Maintain 100 Hz
        sleeper.sleep();
    }
}
