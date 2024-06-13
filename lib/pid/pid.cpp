#include "pid.h"
#include "debug.h"

float PID::update(float value) {
    unsigned long current_time = micros();
    float dt = (current_time - last_update) * 1e-6;

    float error = target - value;
    cumulative_error += error * dt;
    float error_rate = (error - last_error) / dt;
    float output = KP * error + KI * cumulative_error + KD * error_rate;

    //debug_msg("error: %f, XP: %f, XI: %f, XD: %f, out: %f\n", error, error * KP, KI * cumulative_error, KD * error_rate, output);

    last_update = current_time;
    last_error = error;
    
    return output;
}