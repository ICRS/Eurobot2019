// I wrote this node while tripping on magic truffles
#include <ros/ros.h>
#include <eurobot2019_messages/drop_motors.h>
#include <eurobot2019_messages/drop_command.h>
#include <queue>

#include <std_msgs/Int32.h>

#include "message_interface.hpp"
#include "state_manager.hpp"

#define ENOUGH 3 //This is the distance after which the pusher obstructs the stepper (platform on which pucks sit) from moving the tower

// Function to check if floating point numbers are close enough
bool approx_equal(float a, float b) {
    return fabs(a-b) < 1e-2;
}

int main(int argc, char **argv) {
    // init ros
    ros::init(argc, argv, "dropper");

    // grab dat nody boiiii
    ros::NodeHandle nh;


    // Create interface with the hardware node
    MessageInterface<eurobot2019_messages::drop_motors,
                     eurobot2019_messages::drop_motors>
                hardware(10, "drop_motors", 10, "drop_motor_status");
std::cout << "declared hardware" << std::endl;
    // Create interface to the tactics node
    MessageSubscriberInterface<eurobot2019_messages::drop_command>
                command(10, "drop");
                    std::cout << "declared command" << std::endl;
    // sends queue size for every tower
    MessagePublisherInterface<std_msgs::Int32>
                drop_status_left(10, "drop_status_l");
std::cout << "declared drop_left" << std::endl;
    MessagePublisherInterface<std_msgs::Int32>
                drop_status_right(10, "drop_status_r");
std::cout << "declared drop_right" << std::endl;
    MessagePublisherInterface<std_msgs::Int32>
                drop_status_middle(10, "drop_status_m");
std::cout << "declared drop_middle" << std::endl;
    // Create state manager instance
    StateManager state_manager(&nh);

    // Message to be published, and the final target
    eurobot2019_messages::drop_motors motor_msg, target_msg;

    // Set the target message for the dropper
    hardware.set_msg(motor_msg);
    // I think this needs to be deleted

    // Create the old message
    eurobot2019_messages::drop_command old_command_msg;

    // Create the target message for tactics node
    std_msgs::Int32 drop_status_left_msg;
    std_msgs::Int32 drop_status_right_msg;
    std_msgs::Int32 drop_status_middle_msg;

    // Create 3 queues to manage the state order
    std::queue<DropState> state_queue_left;
    state_queue_left.push(IDLE_L);

    std::queue<DropState> state_queue_right;
    state_queue_right.push(IDLE_R);

    std::queue<DropState> state_queue_middle;
    state_queue_middle.push(IDLE_M);

    // Queue is size 1 so set the msg
    drop_status_left_msg.data = 1;
    drop_status_right_msg.data = 1;
    drop_status_middle_msg.data = 1;
    drop_status_left.set_msg(drop_status_left_msg);
    drop_status_right.set_msg(drop_status_right_msg);
    drop_status_middle.set_msg(drop_status_middle_msg);

    // 100 Hz update rate
    ros::Rate sleeper(100);

    while(ros::ok()) {
        // Get the message from command
        auto command_msg = command.get_msg();

        // Do stuff with the message if it has changed
        if(command_msg.left != old_command_msg.left || command_msg.right != old_command_msg.right || command_msg.middle != old_command_msg.middle
           ) {

            ROS_INFO("New command message received");
            old_command_msg = command_msg;

            // Idle command means don't pick up a puck at the moment
            if(command_msg.left == 0) {
                state_queue_left.push(IDLE_L);
            }

            else {
                    state_queue_left.push(RETRACT_PUSHER_L);
                    state_queue_left.push(LOWER_STEPPER_L);
                    state_queue_left.push(EXTEND_PUSHER_L);
                 }

           if(command_msg.right == 0) {
                 state_queue_right.push(IDLE_R);
           }

           else {
                    state_queue_right.push(RETRACT_PUSHER_R);
                    state_queue_right.push(LOWER_STEPPER_R);
                    state_queue_right.push(EXTEND_PUSHER_R);
                }

           if(command_msg.middle == 0) {
                  state_queue_middle.push(IDLE_M);
           }

           else {
                     state_queue_middle.push(RETRACT_PUSHER_M);
                     state_queue_middle.push(LOWER_STEPPER_M);
                     state_queue_middle.push(EXTEND_PUSHER_M);
                 }
           }


           // Update current position
           auto dropper_pos = hardware.get_msg();
           // If the pusher is out (enough to obstruct stepper)
           // And stepper is not at the level of the pusher, retract the pusher
           if(dropper_pos.left_x > ENOUGH &&
              !approx_equal(dropper_pos.left_z,
                            0)) {
               motor_msg.left_x = 0;
           }
           // If target x (pusher position) is not equal to current x, move x
           else if(!approx_equal(target_msg.left_x,
                                 dropper_pos.left_x)) {
               motor_msg.left_x = target_msg.left_x;
           }
           // If target z (stepper position) is not equal to current z, move z
           else if(!approx_equal(target_msg.left_z,
                                 dropper_pos.left_z)) {
               motor_msg.left_z = target_msg.left_z;
           }
           // Then everything is in the correct place, so update the state
           else {
               // Update message before the queue size changes
               // so it include the currently processing one
               drop_status_left_msg.data = state_queue_left.size();
               if(state_queue_left.size()) {
                   target_msg = state_manager.get_target(state_queue_left.front());
                   state_queue_left.pop();
               }
           }

           // repeat for middle tower
           if(dropper_pos.middle_x > ENOUGH &&
              !approx_equal(dropper_pos.middle_z,
                            0)) {
               motor_msg.middle_x = 0;
           }
           else if(!approx_equal(target_msg.middle_x,
                                 dropper_pos.middle_x)) {
               motor_msg.middle_x = target_msg.middle_x;
           }
           else if(!approx_equal(target_msg.middle_z,
                                 dropper_pos.middle_z)) {
               motor_msg.middle_z = target_msg.middle_z;
           }
           else {
               drop_status_middle_msg.data = state_queue_middle.size();
               if(state_queue_middle.size()) {
                   target_msg = state_manager.get_target(state_queue_middle.front());
                   // prev. target_msg is saved as private variable in the state_manager class, so previous changes on target_msg
                   // e.g. for the left tower x and z values are saved, and are kept when target_msg overwritten
                   state_queue_middle.pop();
               }
           }

           // And, for right tower...
           if(dropper_pos.right_x > ENOUGH &&
              !approx_equal(dropper_pos.right_z,
                            0)) {
               motor_msg.right_x = 0;
           }
           else if(!approx_equal(target_msg.right_x,
                                 dropper_pos.right_x)) {
               motor_msg.right_x = target_msg.right_x;
           }
           else if(!approx_equal(target_msg.right_z,
                                 dropper_pos.right_z)) {
               motor_msg.right_z = target_msg.right_z;
           }
           else {
               drop_status_right_msg.data = state_queue_right.size();
               if(state_queue_right.size()) {
                   target_msg = state_manager.get_target(state_queue_right.front());
                   state_queue_right.pop();
               }
           }

           // set messages to hardware (command stepper and pusher) and tactics (number of states left in queue for every tower)
           hardware.set_msg(motor_msg);
           drop_status_left.set_msg(drop_status_left_msg);
           drop_status_right.set_msg(drop_status_right_msg);
           drop_status_middle.set_msg(drop_status_middle_msg);

           // Maintain 100 Hz
           sleeper.sleep();
    }
}

/* //For historical safe-keeping, this is what Nick contributed while high on magical truffles
    // I can see fish peoope

    // Keep going while ROS is a good boi
    while(ros::ok()) {
        // This feels like that time I was awake for 72 hours finishing a deadline coursework coz I'd made a mistake

        My fingers feel funny looool
        // check the state
        switch(crent_state) {
            my cup is really loooooooooooooooong
            he's a long boi



            MY ARM OIS LONG

             and this writing looks like a witch or a tank it's loooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooong

           if the state is right do the thing go away
               semicolon
               how do I end the line
               ;
               ah
               DON'T FORGET THE PLUS SEA


            if(state==EXTENT)
                send_msg(EXTENT)
            if(state==REETRACT_MOTOR)
                do_the_retract();


        }
    }
}
*/
