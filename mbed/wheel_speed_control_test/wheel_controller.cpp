#include "wheel_controller.hpp"
#define RADIUS 10 // in mm (?)

    WheelController::WheelController(PinName pwm, PinName dir,
                    PinName enc_a, PinName enc_b)
    : pwm_(pwm_pin), dir_(dir_pin),
    encoder_(enc_a, enc_b, NC, PULSES_PER_REVOLUTION), // NC would be the PinIndex, but this does not have to be used
    kp(0), ki(0), kd(0), prev_diff_(0), prev_err_(0), integrated_error_(0), prev_rotations(0) {
      // Setup pwm stuff
      // 10000 Hz pwm frequency
      pwm_.period(1.0f/10000.0f);
      pwm_.write(0.0f);

      // Encoder is PULSES_PER_REVOLUTION = 24 ppr (?), and 1 rotation is 2*pi*RADIUS = 20pi mm (?)
      pos_per_pulse_ = 0.00261799387; // meters

      // 0 is clockwise, 1 is anti clockwise
      dir_ = 0;
    }

    void WheelController::update(float dt_s) {
         write_motor_values(dt_s); // cut out? Make dt_s a member variable?
    }

    // Returns the velocity of the wheel
    float WheelController::calculate_velocity(float dt_s){
      // Calculate distance travelled
      float distance_meters = (encoder_.getPulses() - prev_rotations) * pos_per_pulse_;
      // Update previous rotations
      prev_rotations = encoder_.getPulses();
      return current_vel_ = distance_meters/dt_s; // meters per microsecond, multiply by a million for meters per second (I think)
    }

    // Calculate PID returns a result between -1 and 1
    float WheelController::calculate_PID(float dt_s){
      float error = target_vel_ - calculate_velocity(dt_s);
      // Remove sudden jumps in timestep causing surprising results
      if(dt_s > 0.1) dt_s = 0.1; // Is dt_s not always an int?
      // Remove some noise from differentiation
      prev_diff_ = 0.5 * (error - prev_err_)/dt_s + 0.5 * prev_diff_;

      int_err_ += error * dt_s;

      float pid = kp * error + ki * int_error_ + kd * prev_diff_;

      if(pid > 1) pid = 1;
      if(pid < -1) pid = -1;

      // Update previous error
      prev_err_ = error;
      return pid;
    }


    // Write the signed velocity (between -1 and 1) to the motor
    void WheelController::write_motor_values(float dt_s){
    float pid = calculate_PID(dt_s);
    // Set motor values
    if(pid > 0) {
        dir_ = 1;
        // 1 - pid as the dir pin is high
        pwm_.write(1.0f - pid);
    }
    else {
        dir_ = 0;
        pwm_.write(pid);
    }
}
