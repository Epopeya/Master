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
int turn_count = 0;

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

int getTurnSign(int count) {
  int offset = (count < 0) ? 3 : 0;
  int values[] = {1, -1, -1, 1};
  return values[(abs(count) + offset) % 4];
}

float getAxisPosition(int count) {
  switch (abs(count)) {
  case 0: {
    return 0;
  }
  case 1: {
    return 1000;
  }
  case 2: {
    return 2000 * sign(count);
  }
  case 3: {
    return -1000;
  }
  }
  return 0;
}

#define BLOCK_OFFSET
float getCameraOffset(int counter) {
  float offset = 0;
  if (green_block.in_scene) {
    offset = 200;
  } else if (red_block.in_scene) {
    offset = -200;
  }
  return offset * ((counter > 2) ? 1 : -1);
}

float doubleSquare() {
  int mod_turn_count = turn_count % 4;

  float cam_offset = getCameraOffset(mod_turn_count);
  debug_msg("cam: %f", cam_offset);
  
  if (turn_count == 0) { // Maybe remove
    return angleToAxis(position.y, 0);
  }

  int sign = getTurnSign(mod_turn_count);
  float from = (turn_count % 2) ? position.x : position.y;
  float to = (getAxisPosition(mod_turn_count) + cam_offset * sign);

  float angle = turn_count * (PI / 2) + (angleToAxis(from, to) * sign);
  return angle;
}
// from to angle
int last_zone = 0;
int getZone(int turn_sign) {
  if (position.y < 500 && position.y > -500 && position.x < 500) {
    return 0;
  }
  if (position.x > 500 && position.y < 1500 && position.y > -1500) {
    return 1 * turn_sign;
  }
  if (position.x > -500 && (position.y > 1500 || position.y < -1500)) {
    return 2 * turn_sign;
  }
  if (position.x < -500 && (position.y > 500 || position.y < -500)) {
    return 3 * turn_sign;
  }
  return last_zone;
}

void checkBoundaries() {
  if (turn_count == 0 && position.x > 500) {
    // if (left_distance > 1500) {
    if (true) {
      debug_msg("Turn count UP 🥙");
      turn_count++;
      last_zone = 1;
      return;
      //} else if (right_distance > 1500) {
    } else if (true) {
      debug_msg("Turn count DOWN 🫙");
      turn_count--;
      last_zone = -1;
      return;
    } else {
      return;
    }
  }

  int zone = getZone(sign(turn_count));
  if (last_zone != zone) {
    debug_msg("Changing zones: %d->%d 🦔", last_zone, zone);
    turn_count += sign(turn_count);
    debug_msg("Turn count: %d ⛳️", turn_count);
    last_zone = zone;
  }
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
  if (imu.update()) {
    debug_current_direction(imu.rotation);
  }

  int encoders = getEncoders();
  updatePosition(&position, imu.rotation, encoders);
  if (nav_timer.primed()) {
    checkBoundaries();
    servoPid.target = doubleSquare();
    debug_target_direction(servoPid.target);
    servoAngle(servoPid.update(imu.rotation));
  }
}
