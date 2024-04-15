#pragma once
#include <Arduino.h>
#include <debug.h>
#include "position.h"

void slaveSetup();
void servoAngle(int angle);
void motorSpeed(int speed);
void receiveFromSlave();