#pragma once
#include <Arduino.h>
#include <debug.h>

void slaveSetup();
void servoAngle(int angle);
void motorSpeed(int speed);
void receiveFromSlave();
