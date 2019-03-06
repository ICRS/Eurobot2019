#include "wheel_controller.hpp"
#define RADIUS 10

    WheelController::WheelController(PinName pwm, PinName dir,
                    PinName enc_a, PinName enc_b)
    : pwm_(pwm_pin), dir_(dir_pin),
    encoder_(enc_a, enc_b, NC, PULSES_PER_REVOLUTION),
    kp(0), ki(0), kd(0), prev_diff_(0), integrated_error_(0), prev_rotations(0) {
      // Setup pwm stuff
      // 10000 Hz pwm frequency
      pwm_.period(1.0f/10000.0f);
      pwm_.write(0.0f);

      // Encoder is PULSES_PER_REVOLUTION = 24 ppr (?), and 1 rotation is 2*pi*RADIUS = 20pi mm (?)
      pos_per_pulse_ = 0.00261799387;

      // 0 is clockwise, 1 is anti clockwise
      dir_ = 0;
    };

    void WheelController::update(float dt_s) {
         float error = target_vel_ - current_vel_;
         // Remove sudden jumps in timestep causing surprising results
         if(dt_s > 0.1) dt_s = 0.1;
           // Remove some noise from differentiation
         prev_err_ = 0.5*error + 0.5*prev_err_;
         int_err_ += error * dt_s;



    }

    void set_target_vel(float vel_mm){
      if (vel_mm < 0){

      }
      if (vel_mm > 0){

      }
      target_vel_ = vel_mm;
    };

    void get_current_vel(){
      return current_vel_;
    };

    // PID constants
    float kp, ki, kd;

    // Returns the velocity of the wheel
    float calculate_velocity(){
      float rotations = ( (encoder_.getPulses() - prev_rotations) / (float) PULSES_PER_REVOLUTION );

    };

    // Calculate PID returns a result between -1 and 1
float WheelController::calculate_PID(float error, float dt_s){
      float pid = kp * error + ki * int_error_ + kd * prev_err_;
}


    // Write the signed velocity (between -1 and 1) to the motor
void WheelController::write_motor_values(float velocity){
     float velocity= calculate_PID(error, dt_s);
     if(velocity > 1) velocity = 1;
     if(velocity < -1) velocity = -1;

     // Set motor values
     if(velocity > 0) {
         dir_ = 1;
         // 1 - pid as the dir pin is high
         pwm_.write(1.0f - velocity);
     }
     else {
         dir_ = 0;
         pwm_.write(velocity);
}

    // Pins to control the motor via h-bridge
    PwmOut pwm_;
    DigitalOut dir_;

    // Quadrature Encoder Interface to measure the speed of the motor
    QEI encoder_;

    // Integrated error for PID
    float int_err_;

    // Previous error
    float prev_err_;

    // Target velocity
    float target_vel_;

    // Current velocity (for odometry?)
    volatile float current_vel_;
};
