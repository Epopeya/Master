#include "position.h"

MPU9250 mpu;
float rotation = 0;
float target_rotation = 0;
unsigned long gyro_last_time = 0;
unsigned long gyro_dt = 0;
float gyro_offset = 0;
vector2_t orientation = { .x = 1.0, .y = 0 };
vector2_t position = { .x = 0, .y = -750 };

void positionCalibrate() {
  float totalAngle = 0;
  for (int i = 0; i < MPU_CALIBRATION_ITERATIONS; i++) {
    while (!mpu.update()) {}
    totalAngle += mpu.getGyroZ();
  }
  gyro_offset = totalAngle / MPU_CALIBRATION_ITERATIONS;
}

void positionSetup() {
  mpu.setup(MPU_ADDRESS);
}

bool updateGyro() {
  if (mpu.update()) {
    long current_millis = millis();
    gyro_dt = current_millis - gyro_last_time;
    gyro_last_time = current_millis;
    float gyro_value = (mpu.getGyroZ() - gyro_offset) * 2 * PI / 360;
    rotation += gyro_value * gyro_dt / 1000;
    return true;
  }
  return false;
}

float error, last_error, cum_error, rate_error;
float servoPid() {
  error = target_rotation - rotation;
  cum_error += error * gyro_dt;
  rate_error = (error - last_error) / gyro_dt;

  float output = DIR_KP * error + DIR_KI * cum_error + DIR_KD * rate_error;

  last_error = error;

  return output;
}

void positionUpdate() {
  if (updateGyro()) {
    orientation.x = cos(rotation);
    orientation.y = sin(rotation);

    servoAngle(servoPid() * (360.0 / 2.0 * PI) + SERVO_MIDPOINT);
  }
}
