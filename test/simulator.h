#ifndef SIMULATOR_H
#define SIMULATOR_H
#include "serial.h"
void motorSpeed(int speed);
void servoAngle(float angle);
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#endif
