#define SIMULATE_MOVEMENT false


#include <Arduino.h>
#include <debug.h>
#include <geometry.h>
#include <imu.h>
#include <lidar.h>
#include <math.h>
#include <pid.h>
#include <slave.h>
#include <timer.h>

vector2_t position = {.x = 0, .y = 0};
PID servoPid(0.5f, 0.08f, 0.35f);
Imu imu;
Timer nav_timer(20);

void updatePosition(vector2_t *pos, float angle, int encoders) {
  pos->x += encoders * cos(angle) * 1.6;
  pos->y += encoders * sin(angle) * 1.6;
  // debug_msg("X: %f, Y: %f", pos->x, pos->y);
  debug_position(*pos);
}

float angleToAxis(float from, float to) {
  float angle = (to - from) * 0.005;
  return constrain(angle, -(PI / 2), (PI / 2));
}

int last_zone = 0;
int zone = 0;
int turn_count = 0;
float zoneGood(int orientation) {
  float from = (turn_count % 2) ? position.x : position.y;
  float to = from;
  float sign = 0;

  if (position.y < 500 && position.y > -500 && position.x < 500) {
    to = 0;
    sign = 1;
    zone = 0;
  }
  else if (position.x > 500 && position.y < 1500 && position.y > -1500) {
    to = 1000;
    sign = -1 * orientation;
    zone = 1;
  }
  else if (position.x > -500 && (position.y > 1500 || position.y < -1500)) {
    to = 2000 * orientation;
    sign = -1;
    zone = 2;
  }
  else if (position.x < -500 && (position.y > 500 || position.y < -500)) {
    to = -1000;
    sign = 1 * orientation;
    zone = 3;
  }

  float angle = turn_count * (PI / 2) + (angleToAxis(from, to) * sign);

  // imu.rotation = angle;
  if (zone != last_zone) {
    turn_count += orientation;
    debug_msg("zone change to turn: %i", turn_count);
  }
  last_zone = zone;

  return angle;
}

void setup() {
  pinMode(33, INPUT_PULLUP);
  pinMode(26, OUTPUT);

  delay(1000);
  debug_init();
  slaveSetup();
  slaveProcessSerial();
  imu.setup();
  lidarSetup();
  // position = lidarInitialPosition();
  lidarStart();
  debug_msg("Setup completed");

  // // Wait for the user to press the start button
  // digitalWrite(26, HIGH);
  // while (digitalRead(33)) {
  // }
  // digitalWrite(26, LOW);
  // delay(500); // Some time for the user to get their finger out of the way,
  //             // otherwise their finger could easily be cut off in a very
  //             // awful way. Not recommended.
}

Timer batteryTimer(1000);

void loop() {
  if(batteryTimer.primed()) {
    if (battery < 4) {
      motorSpeed(0);
    } else {
      motorSpeed(350);
    }
    debug_battery(battery);
  }
  
  slaveProcessSerial();
  if (SIMULATE_MOVEMENT || imu.update()) {
    debug_current_direction(imu.rotation);
  }

  int encoders = getEncoders();
  if (nav_timer.primed()) {
    if (SIMULATE_MOVEMENT) {
      updatePosition(&position, imu.rotation, 4);
    } else {
      updatePosition(&position, imu.rotation, getEncoders());
    }
    servoPid.target = zoneGood(1);
    debug_target_direction(servoPid.target);
    if (SIMULATE_MOVEMENT) {
      imu.rotation += servoPid.update(imu.rotation) * 0.04;
    } else {
      servoAngle(servoPid.update(imu.rotation));
    }
    // servoAngle(servoPid.update(imu.rotation));
  }
}
