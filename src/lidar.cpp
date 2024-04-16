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
        float r_angle = angle + (target_rotation - rotation) * (360.0f / 2 * PI);
        debug_msg("angle: %f", angle);
        debug_msg("r_angle: %f", r_angle);
        // Serial.println(r_angle);

        // front
        if (r_angle < LIDAR_CHECK_ANGLE || r_angle > 360 - LIDAR_CHECK_ANGLE) {
          front_distance = LIDAR_SMOOTHING * front_distance + LIDAR_INV_SMOOTHING * distance;
        }

        // right
        else if (r_angle < 90 + LIDAR_CHECK_ANGLE && r_angle > 90 - LIDAR_CHECK_ANGLE) {
          right_distance = LIDAR_SMOOTHING * right_distance + LIDAR_INV_SMOOTHING * distance;
        }

        // left
        else if (r_angle < 270 + LIDAR_CHECK_ANGLE && r_angle > 270 - LIDAR_CHECK_ANGLE) {
          left_distance = LIDAR_SMOOTHING * left_distance + LIDAR_INV_SMOOTHING * distance;
        }
      }
    }
  }
}

void lidarSetup() {
  lidar.begin(lidar_serial);
  lidar.startScan();
  xTaskCreatePinnedToCore(lidarTask, "lidar", 100000, NULL, 10, &lidar_task, 0);
  analogWrite(RPLIDAR_MOTOR, 255);
}
