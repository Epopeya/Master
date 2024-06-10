// TODO ENABLE LIDAR AND FLIP Packets
#include <Arduino.h>
#include <MPU9250.h>
#include <debug.h>
#include <vector.h>
#include "esp32-hal.h"
#include "lidar.h"
#include "position.h"
#include "slave.h"

// map
#define OUTER_LENGTH 3000
#define INNER_LENGTH 1700
#define TURNING_POINT 500
#define MIN_TURN_TIME 5500

// waypoint
#define MIN_BLOCK_TIME 300
#define WAYPOINT_MIN_DISTANCE 75
#define WAYPOINTS_SIZE 128

vector2_t waypoints[WAYPOINTS_SIZE] = { { .x = 0, .y = 0 } };
int waypoint_index = 0;
int max_waypoint_index = 0; // exclusive

// timings
unsigned long block_time = 0;
unsigned long last_turn_millis = 0;

#define BATTERY_REPORT_RATE 500
unsigned long last_battery_report = 0;
float battery_voltage = 0.0f;

int total_encoders = 0;

block_t red_block = { 0 };
block_t green_block = { 0 };


float followPoint(vector2_t point) {
  // vector pointing to target
  vector2_t target = {
    .x = point.x - position.x,
    .y = point.y - position.y
  };
  // shortest angle to target
  float angle_diff = atan2(target.y, target.x) - atan2(orientation.y, orientation.x);
  if (angle_diff > PI) {
    angle_diff = angle_diff - 2 * PI;
  } else if (angle_diff < -PI) {
    angle_diff = angle_diff + 2 * PI;
  }

  target_rotation = angle_diff + rotation;

  return sqrt(target.x*target.x + target.y*target.y);
}

void addWaypoint(vector2_t new_point) {
  if (max_waypoint_index == WAYPOINTS_SIZE) {
    debug_msg("Reached max waypoints!!!!");
    return;
  }
  waypoints[max_waypoint_index] = new_point;
  max_waypoint_index++;
}

void nextWaypoint() {
  waypoint_index = (waypoint_index + 1) % max_waypoint_index;
}

void followAndSkipWaypoints(int index = waypoint_index) {
  float dist = followPoint(waypoints[index]);
  if (dist < WAYPOINT_MIN_DISTANCE) {
    nextWaypoint();
  }
}

void setup() {
  //Debugging
  Serial.begin(9600);
  debug_init();

  debug_msg("Entering slaveSetup");
  // // Motor
  slaveSetup();

  debug_msg("Entering position calibration (IMU)");
  Wire.begin();
  positionSetup();
  positionCalibrate();

  servoAngle(SERVO_MIDPOINT);

  waitForBattery();
  if (battery_voltage > 6) {
    lidarSetup();
    motorSpeed(10);
  }
}

int currentTurn = 0;

void loop() {
  Serial.println("main loop");
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

  if (currentTurn < 4){ // first run code
    vector2_t next_waypoint = {
      .x = position.x + orientation.x * WAYPOINT_MIN_DISTANCE,
      .y = position.y + orientation.y * WAYPOINT_MIN_DISTANCE
    };
    float middle_error, middle_adjustment;
    if (front_distance < TURNING_POINT && millis() - last_turn_millis > MIN_TURN_TIME) { // lidar check

      if (left_distance > INNER_LENGTH) {
        if (currentTurn == 0) {
          // debug_map_flip(false);
        }
        absolute_target_rot += PI / 2.0;
        addWaypoint(next_waypoint);
        debug_waypoints(waypoints, max_waypoint_index);
        currentTurn++;
        last_turn_millis = millis();
      } else if (right_distance > INNER_LENGTH) {
        if (currentTurn == 0) {
          // debug_map_flip(true);
        }
        absolute_target_rot -= PI / 2.0;
        addWaypoint(next_waypoint);
        debug_waypoints(waypoints, max_waypoint_index);
        currentTurn++;
        last_turn_millis = millis();
      }
    } else {
      middle_error = left_distance - right_distance;
    // debug_msg("l: %f r: %f\n", left_distance, right_distance);
      middle_adjustment = middle_error * 0;
      debug_msg("adj: %f\n", middle_adjustment);     
    }

    if (green_block.in_scene) {
      relative_target_rot = PI / 3.0f;
      block_time = millis();
    } else if (red_block.in_scene) {
      relative_target_rot = -PI / 3.0f;
      block_time = millis();
    } else if (relative_target_rot != 0 && millis() - block_time > MIN_BLOCK_TIME) {
      addWaypoint(next_waypoint);
      relative_target_rot = 0;
    }

    target_rotation = absolute_target_rot + relative_target_rot + middle_adjustment;

  } else { // waypoint follow code
      if (battery_voltage > 3) {
        motorSpeed(20);
      }
      followAndSkipWaypoints();
  }
}
