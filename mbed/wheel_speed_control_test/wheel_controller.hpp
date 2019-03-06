#ifndef __WHEEL_CONTROLLER_HPP__
#define __WHEEL_CONTROLLER_HPP__

#include "mbed.h"
#include "QEI.h"

class WheelController{
public:
    WheelController(PinName pwm, PinName dir,
                    PinName enc_a, PinName enc_b);
    ~WheelController();

    // Update function to be called regularly by main loop
    void update(float dt_s);

    void set_target_vel(float vel_mm){
      target_vel_ = vel_mm;
    };

    void get_current_vel(){
      retrun current_vel_;
    };

    // PID constants
    float kp, ki, kd;
private:
    // Returns the velocity of the wheel
    float calculate_velocity();

    // Calculate PID returns a result between -1 and 1
    float calculate_PID(float error, float dt_s);

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

#endif // __WHEEL_CONTROLLER_HPP__
