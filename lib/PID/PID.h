#ifndef PID_H
#define PID_H

class PID {
private:
    float kp_;           // Proportional gain
    float ki_;           // Integral gain
    float kd_;           // Derivative gain
    
    float integral_;     // Accumulated integral term
    float previous_error_; // Previous error for derivative calculation
    
    float integral_min_;   // Minimum integral value
    float integral_max_;   // Maximum integral value

public:
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);
    
    // Set gains
    void setGains(float kp, float ki, float kd);
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    
    // Set integral limits
    void setIntegralLimits(float min_val, float max_val);
    
    // Calculate PID output
    float calculate(float setpoint, float current_value, float dt);
    
    // Reset the controller
    void reset();
    
    // Get current gains
    float getKp() const;
    float getKi() const;
    float getKd() const;
};

#endif // PID_H
