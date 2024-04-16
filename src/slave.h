#pragma once
#include <Arduino.h>
#include <debug.h>

extern float battery;

void slaveSetup();
void servoAngle(int angle);
void motorSpeed(int speed);
void receiveFromSlave();
