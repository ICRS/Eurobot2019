#include "wheel_controller.hpp"

class WheelController{

    WheelController(PinName pwm, PinName dir,
                    PinName enc_a, PinName enc_b);
    ~WheelController();

    // Update function to be called regularly by main loop
void WheelController::update(float dt_s) {
     float error = target_vel_ - current_vel_;
     // Remove sudden jumps in timestep causing surprising results
     if(dt_s > 0.1) dt_s = 0.1;
       // Remove some noise from differentiation
     prev_err_ = 0.5*error + 0.5*prev_err_;

     float PID=calculate_PID(error, dt_s);


}



    void set_target_vel(float vel_mm){
      target_vel_ = vel_mm;
    };

    void get_current_vel(){
      return current_vel_;
    };

    // PID constants
    float kp, ki, kd;

    // Returns the velocity of the wheel
    float calculate_velocity();

    // Calculate PID returns a result between -1 and 1
float WheelController::calculate_PID(float error, float dt_s){
      float pid = kp * error + ki * integrated_error_ + kd * prev_diff_;
}


    // Write the signed velocity (between -1 and 1) to the motor
    void write_motor_values(float velocity);

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
