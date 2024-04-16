#include "lidar.h"

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
        float r_angle = angle + (target_rotation - rotation) * (360.0f / (2 * PI));
        // Serial.println(r_angle);

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

  
  vector2_t counter = {.x = 0, .y=0};
  vector2_t start_distances =  {.x = 0, .y=0};
  while (counter.x < 250 && counter.y < 250) {
     if (IS_OK(lidar.waitPoint())) {
        float distance = lidar.getCurrentPoint().distance;  //distance value in mm unit
        float angle = lidar.getCurrentPoint().angle;        //angle value in degrees

        if (!(distance < 10.0 || distance > 3000.0)) {
          if (angle < 5 || angle > 355) {
            counter.x++;
            start_distances.x += distance;
            debug_msg("x_dist: %f", start_distances.x);
          }
          if (angle < 275 && angle > 265) {
            counter.y++;
            start_distances.y += distance;
            debug_msg("y_dist: %f", start_distances.y);

          }
        }
     }
  }
  start_distances.x /= counter.x;
  start_distances.y /= counter.y;

  position.x = 1500 - start_distances.x;
  position.y = -500 - start_distances.y;

  debug_msg("pos.x: %f, pos.y: %f", 
  position.x, position.y);
  

  xTaskCreatePinnedToCore(lidarTask, "lidar", 100000, NULL, 10, &lidar_task, 0);

}
