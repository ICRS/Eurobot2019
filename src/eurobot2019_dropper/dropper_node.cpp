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
                hardware(10, "drop_motors", 10, "drop_status");

    // Create interface to the tactics node
    MessageInterface<std_msgs::Int32, eurobot2019_messages::pickup>
                command(10, "pickup_status", 10, "pickup");

    // Create state manager instance
    StateManager state_manager(&nh);
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
