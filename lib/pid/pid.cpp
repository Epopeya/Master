#include "pid.h"


float PID::update(float error) {
    unsigned long current_time = micros();
    float dt = (current_time - last_update) * 1E-6;

    cumulative_error += error * dt;
    float error_rate = (error - last_error) / dt;
    float output = KP * error + KI * cumulative_error + KD * error_rate;

    last_update = current_time;
    last_error = error;
    
    return output;
}