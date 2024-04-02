#pragma once

#include <RPLidar.h>
#define RPLIDAR_MOTOR 5

extern RPLidarMeasurement lidar_measurement;
extern bool lidar_measurement_available;

void lidarSetup();
