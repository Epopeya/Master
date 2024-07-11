#include "imu.h"
#include <Arduino.h>
#include <MPU9250.h>
#include <EEPROM.h>
#include "debug.h"

MPU9250 mpu;
#define EEPROM_SIZE 1024
unsigned long last_update;
float rotation;

void Imu::setup() {
    Wire.begin();
    EEPROM.begin(EEPROM_SIZE);
    rotation = 0;
    last_update = micros();
    mpu.setup(0x68);

    bool isCalibrated = EEPROM.readByte(0x00);
    if(isCalibrated) {
        float x,y,z = 0;

        x = EEPROM.readFloat(0x01);
        y = EEPROM.readFloat(0x01 + 4);
        z = EEPROM.readFloat(0x01 + 8);
        mpu.setAccBias(x, y, z);

        x = EEPROM.readFloat(0x0D);
        y = EEPROM.readFloat(0x0D + 4);
        z = EEPROM.readFloat(0x0D + 8);
        mpu.setGyroBias(x, y, z);

        x = EEPROM.readFloat(0x19);
        y = EEPROM.readFloat(0x19 + 4);
        z = EEPROM.readFloat(0x19 + 8);
        mpu.setMagBias(x, y, z);

        x = EEPROM.readFloat(0x25);
        y = EEPROM.readFloat(0x25 + 4);
        z = EEPROM.readFloat(0x25 + 8);
        mpu.setMagScale(x, y, z);
    } else {
        debug_msg("WARN: IMU is not calibrated. Loading default values");
        mpu.setAccBias(0., 0., 0.);
        mpu.setGyroBias(0., 0., 0.);
        mpu.setMagBias(0., 0., 0.);
        mpu.setMagScale(1., 1., 1.);
    }

}

bool Imu::update() {
    if(mpu.update()) {
        rotation = mpu.getYaw() * (PI / 180);
        return true;
    } else {
        return false;
    }
}

void Imu::calibrate() {
    mpu.verbose(true);
    debug_msg("Initiating calibration...\nPlease keep the IMU still");
    mpu.calibrateAccelGyro();
    debug_msg("Accel & Gyro calibration completed\nMove the IMU in an eight figure to calibrate magnetometer");
    mpu.calibrateMag();
    debug_msg("Calibration complete\nStoring values to EEPROM");

    EEPROM.writeByte(0x00, 1);
    for(int i = 0; i < 3; i++) {
        EEPROM.writeFloat(0x01 + i*4, mpu.getAccBias(i));
    }
    for(int i = 0; i < 3; i++) {
        EEPROM.writeFloat(0x0D + i*4, mpu.getGyroBias(i));
    }
    for(int i = 0; i < 3; i++) {
        EEPROM.writeFloat(0x19 + i*4, mpu.getMagBias(i));
    }
    for(int i = 0; i < 3; i++) {
        EEPROM.writeFloat(0x25 + i*4, mpu.getMagScale(i));
    }
    EEPROM.commit();
}
