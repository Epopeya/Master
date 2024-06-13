#include "lidar.h"
#include <RPLidar.h>
#include <debug.h>
#include <imu.h>
#include <math.h>

#define RPLIDAR_MOTOR 5
#define SMOOTHING 0.01f // lower = more smoothing
#define INV_SMOOTHING (1 - SMOOTHING)
#define CHECK_ANGLE 10

HardwareSerial lidar_serial(2);
RPLidar lidar;
TaskHandle_t lidar_task;
float front_distance, right_distance, left_distance = 0.0;
extern int turn_count;
extern Imu imu;

void lidarTask(void *pvParameters) {
  for (;;) {
    vTaskDelay(1);
    if (IS_OK(lidar.waitPoint())) {
      RPLidarMeasurement point = lidar.getCurrentPoint();
      float distance = point.distance; // distance value in mm unit
      float angle = point.angle;       // angle value in degrees

      if (!(distance < 10.0 || distance > 3000.0)) {
        float r_angle = angle + (turn_count * (M_PI / 2) - imu.rotation) * (360 / (2 * M_PI));

        // front
        if (r_angle < CHECK_ANGLE || r_angle > 360 - CHECK_ANGLE) {
          front_distance = SMOOTHING * front_distance + INV_SMOOTHING * distance;
        }

        // right
        else if (r_angle < 90 + CHECK_ANGLE && r_angle > 90) {
          right_distance = SMOOTHING * right_distance + INV_SMOOTHING * distance;
        }

        // left
        else if (r_angle < 270 && r_angle > 270 - CHECK_ANGLE) {
          left_distance = SMOOTHING * left_distance + INV_SMOOTHING * distance;
        }
      }
    }
  }
}

void lidarSetup() {
  lidar.begin(lidar_serial);
  lidar.startScan();
  analogWrite(RPLIDAR_MOTOR, 255);
}

#define MEDIAN_ROUNDS 20
vector2_t lidarInitialPosition() {
  vector2_integer_t counter = {.x = 0, .y = 0};
  vector2_t start_distances = {.x = 0, .y = 0};
  while (counter.x < MEDIAN_ROUNDS || counter.y < MEDIAN_ROUNDS) {
    if (IS_OK(lidar.waitPoint())) {
      RPLidarMeasurement point = lidar.getCurrentPoint();
      float distance = point.distance; // distance value in mm unit
      float angle = point.angle;       // angle value in degrees

      if (!(distance < 10.0 || distance > 3000.0)) {
        if (angle < 5 || angle > 355) {
          debug_msg("posX: %f", distance);
          counter.x++;
          start_distances.x += distance;
        }
        if (angle < 275 && angle > 265) {
          debug_msg("posY: %f", distance);
          counter.y++;
          start_distances.y += distance;
        }
      }
    }
  }
  start_distances.x /= MEDIAN_ROUNDS;
  start_distances.y /= MEDIAN_ROUNDS;

  left_distance = start_distances.y;
  right_distance = 1000 - start_distances.y;
  front_distance = start_distances.x;

  return {.x = 1500 - start_distances.x, .y = -500 - start_distances.y};
}

void lidarStart() {
  xTaskCreatePinnedToCore(lidarTask, "lidar", 100000, NULL, 10, &lidar_task, 0);
}
