#include "imu.h"
#include <Arduino.h>
#include <MPU9250.h>
#include <math.h>

MPU9250 mpu;
unsigned long last_update;
float rotation;

int Imu::setup() {
    rotation = 0;
    last_update = micros();
    return mpu.setup(0x68);
}

bool Imu::update() {
    if(mpu.update()){
        unsigned long current_time = micros();
        unsigned long dt = current_time - last_update;
        rotation += mpu.getGyroZ() * dt * 1e-6 * (M_PI / 180);
        return true;
    } else {
        return false;
    }
}
