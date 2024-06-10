#pragma once
#include <Arduino.h>
#include <MPU9250.h>
#include "vector.h"
#include "slave.h"

#define MPU_ADDRESS 0x68
#define MPU_CALIBRATION_ITERATIONS 1000
#define MILIMETERS_PER_ENCODER 3.86f
#define SERVO_MIDPOINT 92

#define DIR_KP 0.1
#define DIR_KI 0
#define DIR_KD 0.02


extern float rotation;
extern float target_rotation;
extern float relative_target_rot;
extern float absolute_target_rot;

extern vector2_t position;
extern vector2_t orientation;

void positionSetup();
void positionCalibrate();
void positionUpdate();
