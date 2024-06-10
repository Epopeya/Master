#include "lidar.h"

#define LIDAR_INITIAL_POS_ROUNDS 20

HardwareSerial lidar_serial(2);
RPLidar lidar;
TaskHandle_t lidar_task;


float front_distance, right_distance, left_distance = 0.0;

void lidarTask(void *pvParameters) {
  while (true) {
    vTaskDelay(1);  //TODO: Is this really needed?
    if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance;  //distance value in mm unit
      float angle = lidar.getCurrentPoint().angle;        //angle value in degrees

      if (!(distance < 10.0 || distance > 3000.0)) {
        float r_angle = angle + (absolute_target_rot - rotation) * (360.0f / (2 * PI));

        // TODO: Doesn't account for a rotated robot
        float abs_angle = rotation - angle * 2 * PI / 360;
        // debug_lidar({ .x = position.x + distance * cosf(abs_angle),
        //               .y = position.y + distance * sinf(abs_angle) });
        // front
        if (r_angle < LIDAR_CHECK_ANGLE || r_angle > 360 - LIDAR_CHECK_ANGLE) {
          front_distance = LIDAR_SMOOTHING * front_distance + LIDAR_INV_SMOOTHING * distance;
        }

        // right
        else if (r_angle < 90 + LIDAR_CHECK_ANGLE && r_angle > 90) {
          right_distance = LIDAR_SMOOTHING * right_distance + LIDAR_INV_SMOOTHING * distance;
        }

        // left
        else if (r_angle < 270 && r_angle > 270 - LIDAR_CHECK_ANGLE) {
          left_distance = LIDAR_SMOOTHING * left_distance + LIDAR_INV_SMOOTHING * distance;
        }
      }
    }
  }
}

void lidarSetup() {
  lidar.begin(lidar_serial);
  lidar.startScan();
  analogWrite(RPLIDAR_MOTOR, 255);

  vector2_t counter = { .x = 0, .y = 0 };
  vector2_t start_distances = { .x = 0, .y = 0 };
  while (counter.x < LIDAR_INITIAL_POS_ROUNDS || counter.y < LIDAR_INITIAL_POS_ROUNDS) {
    if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance;  //distance value in mm unit
      float angle = lidar.getCurrentPoint().angle;        //angle value in degrees

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
  start_distances.x /= counter.x;
  start_distances.y /= counter.y;

  position.x = 1500 - start_distances.x;
  position.y = -500 - start_distances.y;

  left_distance = start_distances.y;
  right_distance = 1000 - start_distances.y;
  front_distance = start_distances.x;

  debug_msg("pos.x: %f, pos.y: %f",
            position.x, position.y);

  xTaskCreatePinnedToCore(lidarTask, "lidar", 100000, NULL, 10, &lidar_task, 0);
}
