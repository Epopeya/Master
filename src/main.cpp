#include <Arduino.h>
#include <debug.h>
#include <slave.h>
#include <geometry.h>
#include <imu.h>
#include <pid.h>
#include <math.h>

vector2_t position = {.x = 0, .y = -1000};
PID servoPid(0.7f, 0.1f, 0.35f);
Imu imu;

void updatePosition(vector2_t *pos, float angle, int encoders) {
    pos->x += encoders * cos(angle);
    pos->y += encoders * sin(angle);
    debug_position(*pos);
}

float axisError(vector2_t pos, int turn, int clocksise) {
  float target = -1000;
  float current = pos.y;

  if (turn % 2 == 1) {
    current = pos.x;
  }

  float error = target - current;

  if (turn > 1) {
    error *= -1;
  }
  return error;
}

void square() {
  float error = axisError(position, 0, 1);
  float servoValue = servoPid.update(error);

  debug_msg("err: %f, ser: %f", error, servoValue);

  servoAngle(servoValue);
}


void setup() {
  delay(5000);
  debug_init();
  slaveSetup();
  
  imu.setup();
  debug_msg("Setup completed");

  servoAngle(0.0f);
  motorSpeed(220);
}

void loop() {
  slaveProcessSerial();
  imu.update();
  updatePosition(&position, imu.rotation, getEncoders());

  square();

  delay(20);
}