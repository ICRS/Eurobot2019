#include "mbed.h"
#include "AsyncStepper.hpp"

int main() {
    
    AsyncStepper stepper(p15, p16, p17, 60);

    while(true) {
        stepper.Rotate(direction_e::POSITIVE, 100);
        wait(1);
        stepper.Rotate(direction_e::NEGATIVE, 100);
        wait(1);
    }

    return 0;
}
