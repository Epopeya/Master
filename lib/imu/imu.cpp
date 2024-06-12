#include "imu.h"
#include <Arduino.h>
#include <MPU9250.h>
#include <math.h>
#include "debug.h"

MPU9250 mpu;
unsigned long last_update;
float rotation;


#define CALIBRATION_ITER 500

void Imu::setup() {
    Wire.begin();
    rotation = 0;
    last_update = micros();
    mpu.setup(0x68);

    // Calibrate initial offset
    float totalAngle = 0;
    for (int i = 0; i < CALIBRATION_ITER; i++) {
        while (!mpu.update()) {}
        totalAngle += mpu.getGyroZ();
    }
    offset = totalAngle / CALIBRATION_ITER;
}

bool Imu::update() {
    if(mpu.update()) {
        unsigned long current_time = micros();
        float dt = (current_time - last_update) * 1e-6;
        rotation += (mpu.getGyroZ() - offset) * dt * (M_PI / 180);
        last_update = current_time;
        return true;
    } else {
        return false;
    }
}
