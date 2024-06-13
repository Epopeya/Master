#include <Arduino.h>
#include <debug.h>
#include <geometry.h>
#include <imu.h>
#include <lidar.h>
#include <math.h>
#include <pid.h>
#include <slave.h>
#include <timer.h>

vector2_t position = {.x = -1000, .y = -1000};
PID servoPid(0.5f, 0.15f, 0.25f);
Imu imu;
Timer nav_timer(20);
int turn_count = 0;

void updatePosition(vector2_t *pos, float angle, int encoders) {
  pos->x += encoders * cos(angle) * 3.86f;
  pos->y += encoders * sin(angle) * 3.86f;
  // debug_msg("X: %f, Y: %f", pos->x, pos->y);
  debug_position(*pos);
}

float angleToAxis(vector2_t pos/* int height, bool vertical*/) {
  return ((-1000) - pos.y) * 0.0005;
}

int turn = 0;
int clockwise = 1;
void square() {
  servoPid.target = angleToAxis(position);
  float servoValue = servoPid.update(imu.rotation);
  //servoValue = constrain(servoValue, minAngle, maxAngle);
  // debug_msg("err: %f, ser: %f", error, servoValue);
  servoAngle(servoValue);
}

void setup() {
  delay(1000);
  debug_init();
  slaveSetup();
  imu.setup();
  // lidarSetup();
  // position = lidarInitialPosition();
  // lidarStart();
  debug_msg("Setup completed");

  servoAngle(0.0f);
  motorSpeed(3);
}

void loop() {
  slaveProcessSerial();
  if (imu.update()) {
    debug_current_direction(imu.rotation);
  }

  int encoders = getEncoders();
  updatePosition(&position, imu.rotation, encoders);

  if (nav_timer.primed()) {
    square();
  }
  motorSpeed(3);
}
