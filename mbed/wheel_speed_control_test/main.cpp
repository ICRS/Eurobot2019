#include "mbed.h"
#include "wheel_controller.hpp"

Serial pc(USBTX, USBRX);

int main() {
  // inputs for WheelController object (Pin numbers) in order:
  // pwm, dir, enc_a, enc_b
    WheelController controller(PB_5, PB_4, PF_0, PF_1);

    // Set PID values, change and test
    controller.kp = 2;
    controller.ki = 2;
    controller.kd = 0.03;


    // Time keeping
    Timer timer;
    timer.start();
    float dt;

    pc.baud(115200);

    controller.set_target_vel(0.6);

    while(true) {
      // Update the PID controller
      dt = timer.read();
      timer.reset();

      controller.update(dt);

      pc.printf("Speed: %f, dt: %f\r\n", controller.get_current_vel(), dt);

      wait_ms(10);
    }

    return 0;
}
