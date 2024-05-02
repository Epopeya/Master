#include <Arduino.h>
#include <MPU9250.h>
#include <debug.h>
#include <vector.h>
#include "esp32-hal.h"
#include "lidar.h"
#include "position.h"
#include "slave.h"

#define OUTER_LENGTH 3000
#define INNER_LENGTH 1700
#define TURNING_POINT 600
#define MIN_TURN_TIME 4000
#define WAYPOINT_MIN_DISTANCE 50

#define WAYPOINTS_SIZE 4
vector2_t waypoints[WAYPOINTS_SIZE] = { { .x = 0, .y = 0 } };
int waypoint_index = 0;
unsigned long last_turn_millis = 0;

#define BATTERY_REPORT_RATE 500
unsigned long last_battery_report = 0;
float battery_voltage = 0.0f;

int total_encoders = 0;

block_t red_block = { 0 };
block_t green_block = { 0 };

// calculate new direction and change waypoints
void followWaypoint() {
  // vector pointing to target
  vector2_t target = {
    .x = waypoints[waypoint_index].x - position.x,
    .y = waypoints[waypoint_index].y - position.y
  };

  // shortest angle to target
  float angle_diff = atan2(target.y, target.x) - atan2(orientation.y, orientation.x);
  if (angle_diff > PI) {
    angle_diff = angle_diff - 2 * PI;
  } else if (angle_diff < -PI) {
    angle_diff = angle_diff + 2 * PI;
  }

  target_rotation = angle_diff + rotation;

  if (sqrt(target.x * target.x + target.y * target.y) < WAYPOINT_MIN_DISTANCE) {
    waypoint_index = (waypoint_index + 1) % WAYPOINTS_SIZE;
  }
}

void setup() {
  delay(3000);
  // Motor
  slaveSetup();

  Wire.begin();
  positionSetup();
  positionCalibrate();


  servoAngle(SERVO_MIDPOINT);

  //Debugging
  Serial.begin(9600);
  debug_init();
  //gyro_last_time = millis();
  waitForBattery();
  if (battery_voltage > 6) {
    //lidarSetup();
    motorSpeed(0);
  }
}

int currentTurn = 0;

void loop() {
  // update direction
  positionUpdate();

  receiveFromSlave();

  if (millis() - last_battery_report > BATTERY_REPORT_RATE) {
    debug_battery(battery_voltage);
    last_battery_report = millis();
  };

  debug_position(position);
  debug_current_direction(rotation);
  debug_target_direction(target_rotation);

  if (currentTurn >= 4) {
    if (battery_voltage > 3) {
      motorSpeed(30);
    }
    followWaypoint();
  } else if (front_distance < TURNING_POINT && millis() - last_turn_millis > MIN_TURN_TIME) {
    if (left_distance > INNER_LENGTH) {
      if (currentTurn == 0) {
        debug_map_flip(false);
      }
      target_rotation += PI / 2.0;
      waypoints[currentTurn].x = position.x + orientation.x * 250;
      waypoints[currentTurn].y = position.y + orientation.y * 250;

      debug_waypoints(waypoints, currentTurn + 1);
      currentTurn++;
      last_turn_millis = millis();
    } else if (right_distance > INNER_LENGTH) {
      if (currentTurn == 0) {
        debug_map_flip(true);
      }
      target_rotation -= PI / 2.0;
      waypoints[currentTurn].x = position.x;
      waypoints[currentTurn].y = position.y;

      debug_waypoints(waypoints, currentTurn + 1);
      currentTurn++;
      last_turn_millis = millis();
    }
  }
}
