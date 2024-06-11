#include "pid.h"


float PID::update() {
    unsigned long current_time = micros();
    float error = target - last_value;
    unsigned long dt = current_time - last_update;

    cumulative_error += error * dt;
    float error_rate = (error - last_error) / dt;
    float output = KP * error + KI * cumulative_error + KD * error_rate;

    last_update = current_time;
    last_error = error;
    
    return output;
}