#include <ros/ros.h>
#include <eurobot2019_messages/grabber_motors.h>
#include <eurobot2019_messages/pickup.h>
#include <queue>

#include <std_msgs/Int32.h>
#include <std_msgs/Int8MultiArray.h>

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
    MessageInterface<std_msgs::Int8MultiArray
                     std_msgs::Int8MultiArray>
                hardware(10, "grabber_command", 10, "grabber_status");

    // Create interface to the tactics node
    MessageInterface<std_msgs::Int32, eurobot2019_messages::pickup>
                command(10, "pickup_status", 10, "pickup");

    // Create state manager instance
    StateManager state_manager(&nh);

    // Message to be published, and the final target
    std_msgs::Int8MultiArray motor_msg, target_msg;

    // Set the target message for the grabber
    hardware.set_msg(motor_msg);

    // Create the old message
    eurobot2019_messages::pickup old_command_msg;

    // Create the target message for tactics node
    std_msgs::Int32 pickup_status_msg;

    // Create a queue to manage the state order
    std::queue<GrabberStates> state_queue;
    state_queue.push(IDLE_BACK);

    // Queue is size 1 so set the msg
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
        // If the current twist is non zero
        // And current z is not at the target, set twist to 0
        if(grabber_pos.z_twist_rad > 1e-2 &&
           !approx_equal(target_msg.z_pos_mm,
                         grabber_pos.z_pos_mm)) {
            motor_msg.z_twist_rad = 0;
        }
        // If target z is not equal and twist is 0, move z
        else if(!approx_equal(target_msg.z_pos_mm,
                              grabber_pos.z_pos_mm)) {
            motor_msg.z_pos_mm = target_msg.z_pos_mm;
        }
        // If z is correct but twist is not, move twist
        else if(!approx_equal(target_msg.z_twist_rad,
                              grabber_pos.z_twist_rad)) {
            motor_msg.z_twist_rad = target_msg.z_twist_rad;
        }
        // If z and twist are in the correct position, adjust open/close
        else if(!approx_equal(target_msg.open_pos_mm,
                              grabber_pos.open_pos_mm)) {
            motor_msg.open_pos_mm = target_msg.open_pos_mm;
        }
        // Finally if the servo twist is wrong, update that
        else if(target_msg.servo_state != grabber_pos.servo_state) {
            motor_msg.servo_state = target_msg.servo_state;
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
        command.set_msg(pickup_status_msg);

        // Maintain 100 Hz
        sleeper.sleep();
    }
}
