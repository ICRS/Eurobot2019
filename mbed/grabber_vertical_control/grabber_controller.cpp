#include "grabber_controller.h"

// There are 4 possible encoder states, which are as follows
// The format is 0b000000AB in the code
const char _encoder_states_[4] = {
    0b00,
    0b01,
    0b11,
    0b10
};

GrabberController::GrabberController(PinName pwm_pin, PinName dir_pin,
                                     PinName encoder_a, PinName encoder_b)
        : pwm_(pwm_pin), dir_(dir_pin), 
          encoder_a_(encoder_a), encoder_b_(encoder_b),
          kp(0), ki(0), kd(0), prev_diff_(0), integrated_error_(0) {
    // Setup pwm stuff
    // 10000 Hz pwm frequency
    pwm_.period(1.0f/10000.0f);
    pwm_.write(0.0f);

    // Encoder is 24 ppr, and 1 rotation is 20pi mm
    pos_per_pulse_ = 0.00261799387;

    // 0 is clockwise, 1 is anti clockwise
    dir_ = 0;
    
    // Set previous encoder state
    prev_encoder_state_ = (encoder_a_ << 1) | encoder_b_;
    for(char i = 0; i < 4; i++) {
        if(prev_encoder_state_ == _encoder_states_[i]) {
            prev_encoder_state_ = i;
            break;
        }
    }

    // Set the encoder interrupt to trigger on a rising or falling edge
    // from channel a or b
    encoder_a_.rise(callback(this, &GrabberController::encoder_interrupt));
    encoder_a_.fall(callback(this, &GrabberController::encoder_interrupt));
    encoder_b_.rise(callback(this, &GrabberController::encoder_interrupt));
    encoder_b_.fall(callback(this, &GrabberController::encoder_interrupt));

    // Wait for startup command
    disable();
}

void GrabberController::update(float dt_s) {
    if(!enabled_) return;

    float error = target_pos_ - current_pos_;
    
    // Remove sudden jumps in timestep causing surprising results
    if(dt_s > 0.1) dt_s = 0.1;

    // Increase integrated error
    integrated_error_ = error * dt_s;

    // Remove some noise from differentiation
    prev_diff_ = 0.5 * error/dt_s + 0.5 * prev_diff_;

    float pid = kp * error + ki * integrated_error_ + kd * prev_diff_;

    // Limit values to +- 1.0 (setting duty cycle)
    if(pid > 1) pid = 1;
    if(pid < -1) pid = -1;

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

void GrabberController::enable() {
    enabled_ = true;
}

void GrabberController::disable() {
    // Disable pwm output
    pwm_.write(0);
    dir_ = 0;
    enabled_ = false;
}

void GrabberController::encoder_interrupt() {
    // Calculate current state
    char state = (encoder_a_ << 1) | encoder_b_;

    // Workout current state
    char i;
    for(i = 0; i < 4; i++)
        if(state == _encoder_states_[i]) break;

    char measured_dir;

    // Compare with previous state to get direction
    if(prev_encoder_state_ == (i+1) % 4) measured_dir = 1;
    else measured_dir = 0;

    // Update previous state
    prev_encoder_state_ = i;

    // Increment position
    if(measured_dir)
        current_pos_ += pos_per_pulse_;
    else
        current_pos_ -= pos_per_pulse_;
}
