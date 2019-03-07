#include "mbed.h"
#include "wheel_controller.hpp"

int main() {
  // inputs for WheelController object (Pin numbers) in order:
  // pwm, dir, enc_a, enc_b
    WheelController controller(NC, NC, NC, NC);

    // Set PID values, change and test
    controller.kp = 0.1;
    controller.ki = 0;
    controller.kd = 0;

    // Time keeping
    Timer timer;
    timer.start();
    float dt;

    while(true) {
      // Update the PID controller
      dt = timer.read();
      timer.reset();

      controller.update(dt);
    }

    return 0;
}
