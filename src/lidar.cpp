#include "lidar.h"

HardwareSerial lidar_serial(2);
RPLidar lidar;
TaskHandle_t lidar_task;

RPLidarMeasurement lidar_measurement;
bool lidar_measurement_available;

void lidarTask(void *pvParameters) {
  while (true) {
    vTaskDelay(1);  //TODO: Is this really needed?
    if (IS_OK(lidar.waitPoint())) {
      lidar_measurement = lidar.getCurrentPoint();
      lidar_measurement_available = true;
    }
  }
}

void lidarSetup() {
  lidar.begin(lidar_serial);
  lidar.startScan();
  xTaskCreatePinnedToCore(lidarTask, "lidar", 100000, NULL, 10, &lidar_task, 0);
  analogWrite(RPLIDAR_MOTOR, 255);
}
