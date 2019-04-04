// I wrote this node while tripping on magic truffles

#include <ros/ros.h>
#include <message_interface.hpp>

enum DropState {
    IDLE,
    LOWER_STEPPER,
    RETRACT_PUSHER,
    EXTEND_PUSHER
};

int main(int argc, char **argv) {
    // init ros
    ros::init(argc, argv, "dropper");

    // grab dat nody boiiii
    ros::NodeHandle nh;

    // Create interface with the hardware node
    MessageInterface<eurobot2019_messages::drop_motors,
                     eurobot2019_messages::drop_motors>
                hardware(10, "drop_motors", 10, "drop_motor_status");

    // Create interface to the tactics node
    MessageInterface<std_msgs::Int32, eurobot2019_messages::drop_command>
                command(10, "drop_status", 10, "drop");

    // Create state manager instance
    StateManager state_manager(&nh);

    // Message to be published, and the final target
    eurobot2019_messages::drop_motors motor_msg, target_msg;

    // Set the target message for the grabber
    hardware.set_msg(motor_msg);

    // Create the old message
    eurobot2019_messages::drop_command old_command_msg;

    // Create the target message for tactics node
    std_msgs::Int32 drop_status_msg;

    // Create 3 queues to manage the state order
    std::queue<DropState> state_queue_left;
    state_queue_left.push(IDLE);

    std::queue<DropState> state_queue_right;
    state_queue_right.push(IDLE);

    std::queue<DropState> state_queue_middle;
    state_queue_middle.push(IDLE);

    // Queue is size 1 so set the msg
    drop_status_msg.data = 1;
    command.set_msg(drop_status_msg);

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
                state_queue_left.push(IDLE);
            }

            else {
                    state_queue_left.push(RETRACT_PUSHER);
                    state_queue_left.push(LOWER_STEPPER);
                    state_queue_left.push(EXTEND_PUSHER);
                 }

           if(command_msg.right == 0) {
                 state_queue_right.push(IDLE);
           }

           else {
                    state_queue_right.push(RETRACT_PUSHER);
                    state_queue_right.push(LOWER_STEPPER);
                    state_queue_right.push(EXTEND_PUSHER);
                }

           if(command_msg.middle == 0) {
                  state_queue_middle.push(IDLE);
           }

           else {
                     state_queue_middle.push(RETRACT_PUSHER);
                     state_queue_middle.push(LOWER_STEPPER);
                     state_queue_middle.push(EXTEND_PUSHER);
                 }
               }
             }


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
