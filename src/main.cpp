#include <Arduino.h>
#include <debug.h>
#include <slave.h>
#include <geometry.h>
#include <imu.h>
#include <pid.h>
#include <math.h>

vector2_t position = {.x = 1500, .y = 500};

void updatePosition(vector2_t *pos, float angle, int encoders) {
    pos->x += encoders * cos(angle);
    pos->y += encoders * sin(angle);
    debug_position(*pos);
}

Imu imu;

void setup() {
  delay(5000);
  debug_init();
  slaveSetup();
  
  imu.setup();
  debug_msg("Setup completed");

  servoAngle(90);
  motorSpeed(10);
}

PID servoPid(1, 0, 0);

void loop() {
  slaveProcessSerial();
  imu.update();
  updatePosition(&position, imu.rotation, getEncoders());

  servoPid.target = imu.rotation - angleBetweenPoints(position, {.x = 2500, .y = 500});
  servoAngle(servoPid.update());
  delay(20);
}