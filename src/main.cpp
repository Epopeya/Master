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
  return constrain(angle, -(PI / 3), (PI / 3));
}

int turn = 0;
int clockwise = 1;
void square() {
  if(position.x > 500 && (turn_count % 4) == 0) {
    debug_msg("Turning 👍");
    turn_count++;
  } else if(position.y > 500 && (turn_count % 4) == 1) {
    debug_msg("Turning 😋");
    turn_count++;
  } else if(position.x < -500 && (turn_count % 4) == 2) {
    debug_msg("Turning 🥒");
    turn_count++;
  } else if(position.y < -500 && (turn_count % 4) == 3) {
    debug_msg("Done 🚀🔥🦙");
    turn_count++;
  }
  float angle = turn_count * (PI / 2);
  switch (turn_count % 4) {
    case 0: {
      servoPid.target = angle + angleToAxis(position.y, -1000);
      break;
    }
    case 1: {
      servoPid.target = angle - angleToAxis(position.x, 1000);
      break;
    }
    case 2: {
      servoPid.target = angle - angleToAxis(position.y, 1000);
      break;
    }
    case 3: {
      servoPid.target = angle + angleToAxis(position.x, -1000);
      break;
    }
  }
    
  
  debug_target_direction(servoPid.target);  
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
  motorSpeed(6);
}
