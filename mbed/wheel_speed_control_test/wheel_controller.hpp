#ifndef __WHEEL_CONTROLLER_HPP__
#define __WHEEL_CONTROLLER_HPP__

#include "mbed.h"
#include "QEI.h"

#define PULSES_PER_REVOLUTION 24

class WheelController{
public:
    WheelController(PinName pwm, PinName dir,
                    PinName enc_a, PinName enc_b);
    ~WheelController();

    // Update function to be called regularly by main loop
    void update(float dt_s);

    void set_target_vel(float vel_m){
      target_vel_ = vel_m;
    };

    // Current velocity (for odometry?)
    void get_current_vel(){
      return current_vel_;
    };

    // PID constants
    float kp, ki, kd;
private:
    // Returns the velocity of the wheel
    float calculate_velocity(float dt_s);

    // Calculate PID returns a result between -1 and 1
    float calculate_PID(float dt_s);

    // Write the signed velocity (between -1 and 1) to the motor
    void write_motor_values(float dt_s);

    // Pins to control the motor via h-bridge
    PwmOut pwm_;
    DigitalOut dir_;

    // Quadrature Encoder Interface to measure the speed of the motor
    // NC would be  the PinIndex, but this does not have to be used
    QEI encoder_;

    // Integrated error for PID
    float int_err_;

    // Previous error
    float prev_err_;

    // Previous rate of change of error
    float prev_diff_;

    // Previous pulses ---> rotations
    float prev_rotations;

    // Target velocity
    float target_vel_;

    // Current velocity (for odometry?)
    volatile float current_vel_;
};

#endif // __WHEEL_CONTROLLER_HPP__
