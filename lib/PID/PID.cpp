#include "PID.h"
#include <algorithm>

PID::PID(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd),
      integral_(0.0f), previous_error_(0.0f),
      integral_min_(-255.0f), integral_max_(255.0f) {
}

void PID::setGains(float kp, float ki, float kd) {
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;
}

void PID::setKp(float kp) {
    this->kp_ = kp;
}

void PID::setKi(float ki) {
    this->ki_ = ki;
}

void PID::setKd(float kd) {
    this->kd_ = kd;
}

void PID::setIntegralLimits(float min_val, float max_val) {
    integral_min_ = min_val;
    integral_max_ = max_val;
}

float PID::calculate(float target_value, float current_value, float dt) {
    // Calculate error
    float error = target_value - current_value;
    
    // Proportional term
    float p_term = kp_ * error;
    
    // Integral term
    integral_ += error * dt;
    // Clamp integral to limits
    integral_ = std::max(integral_min_, std::min(integral_max_, integral_));
    float i_term = ki_ * integral_;
    
    // Derivative term
    float d_term = 0.0f;
    if (dt > 0.0f) {
        d_term = kd_ * (error - previous_error_) / dt;
    }
    previous_error_ = error;
    
    // Calculate total output
    float output = p_term + i_term + d_term;
    
    return output;
}

void PID::reset() {
    integral_ = 0.0f;
    previous_error_ = 0.0f;
}

float PID::getKp() const {
    return kp_;
}

float PID::getKi() const {
    return ki_;
}

float PID::getKd() const {
    return kd_;
}
