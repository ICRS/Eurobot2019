#include "mbed.h"
#include "grabber_controller.h"

#define I2C_ADDR 0xC4


/****************
 * I2C commands *
 ****************/

// Set commands - set a variable

#define SET_POS 0x01 // Set target pos
#define SET_KP 0x02 // Set kp
#define SET_KI 0x03 // Set ki
#define SET_KD 0x04 // Set kd

#define READ_POS 0x10 // Read current pos
#define READ_KP 0x11 // Read kp
#define READ_KI 0x12 // Read ki
#define READ_KD 0x13 // Read kd

#define ENABLE 0xFE // Turns on the controller
#define DISABLE 0xFF // Turns off the controller

I2CSlave slave(NC, NC);

int main() {
    GrabberController controller(NC, NC, NC, NC);
    
    char buf[10];
    char next_read = READ_POS;

    // Flag to check whether there is data to be processed
    bool read_data = false;

    // Time keeping
    Timer timer;
    timer.start();
    float dt;
    
    while(true) {
        // Process I2C slave messages
        int i = slave.receive();
        switch(i) {
        case I2CSlave::ReadAddressed:
            memset(buf, '\0', sizeof(buf));
            switch(next_read) {
            case READ_POS:
                sprintf(buf, "%f", controller.get_current_pos());
                break;
            case READ_KP:
                sprintf(buf, "%f", controller.kp);
                break;
            case READ_KI:
                sprintf(buf, "%f", controller.ki);
                break;
            case READ_KD:
                sprintf(buf, "%f", controller.kd);
                break;
            }
            slave.write(buf, strlen(buf) + 1); // Includes null char
            break;
        case I2CSlave::WriteAddressed:
            slave.read(buf, sizeof(buf));
            read_data = true;
            break;
        case I2CSlave::WriteGeneral:
            slave.read(buf, sizeof(buf));
            read_data = true;
            break;
        }
// Process data if it has been read
        if(read_data) {
            char data[strlen(buf)];
            switch(buf[0]) {
            case SET_POS:
                sprintf(data, "%s", buf+1);
                controller.set_target_pos(atof(data));
                break;
            case SET_KP:
                sprintf(data, "%s", buf+1);
                controller.kp = atof(data);
                break;
            case SET_KI:
                sprintf(data, "%s", buf+1);
                controller.ki = atof(data);
                break;
            case SET_KD:
                sprintf(data, "%s", buf+1);
                controller.kd = atof(data);
                break;

            case READ_POS:
            case READ_KP:
            case READ_KI:
            case READ_KD:
                next_read = buf[0];
                break;

            case ENABLE:
                controller.enable();
                break;
            case DISABLE:
                controller.disable();
                break;
            }
        }

        // Update the PID controller
        dt = timer.read();
        timer.reset();

        controller.update(dt);
    }

    return 0;
}
