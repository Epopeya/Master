#include <Arduino.h>
#include <debug.h>
#include <slave.h>
#include <geometry.h>
#include <imu.h>
#include <pid.h>
#include <math.h>
#include <timer.h>

vector2_t position = {.x = 0, .y = -1000};
PID servoPid(0.05f, 0.01f, 0.015f);
Imu imu;
Timer nav_timer(20);

void updatePosition(vector2_t *pos, float angle, int encoders) {
    pos->x += encoders * cos(angle) * 3.86f;
    pos->y += encoders * sin(angle) * 3.86f;
    // debug_msg("X: %f, Y: %f", pos->x, pos->y);
    debug_position(*pos);
}

float axisError(vector2_t pos, int turn, int clockwise) {
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

int turn = 0;
int clockwise = 1;
void square() {
  float error = axisError(position, turn, clockwise);
  float targetAngle = turn * (PI/2) * clockwise; // -1 if clockwise

  float angleDiff = targetAngle - imu.rotation;
  float minAngle = -(PI / 2) + angleDiff;
  float maxAngle = (PI / 2) - angleDiff;

  // debug_msg("min: %f, max %f", minAngle, maxAngle);

  float servoValue = servoPid.update(error);
  debug_msg("err: %f, ser: %f", error, servoValue);

  servoAngle(servoValue);
}


void setup() {
  delay(1000);
  debug_init();
  slaveSetup();
  
  imu.setup(); 
  debug_msg("Setup completed");

  servoAngle(0.0f);
  motorSpeed(3);
}

void loop() {
  slaveProcessSerial();
  if(imu.update()) {
    debug_current_direction(imu.rotation);
  }



  int encoders = getEncoders();
  updatePosition(&position, imu.rotation, encoders);

  if (nav_timer.primed()) {
    square();
  }
  motorSpeed(3);
}