#ifndef __GRABBER_CONTROLLER_H__
#define __GRABBER_CONTROLLER_H__

#include "mbed.h"

class GrabberController {
public:
    GrabberController(PinName pwm_pin, PinName dir_pin, 
                      PinName encoder_a, PinName encoder_b);

    void set_target_pos(float position_mm) { 
        target_pos_ = position_mm; 
    };

    float get_current_pos() {
        return current_pos_;
    };

    void update(float dt_s);

    void enable();

    void disable();
    
    float kp, ki, kd;
private:
    bool enabled_;

    float pos_per_pulse_;

    float prev_diff_;
    float integrated_error_;

    float target_pos_;
    volatile float current_pos_;
    
    volatile char prev_encoder_state_;

    InterruptIn encoder_a_;
    InterruptIn encoder_b_;

    PwmOut pwm_;
    DigitalOut dir_;

    void encoder_interrupt();
};

#endif // __GRABBER_CONTROLLER_H__
