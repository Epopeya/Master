#pragma once
#include <RPLidar.h>
#include "position.h"
#define RPLIDAR_MOTOR 5
#define LIDAR_SMOOTHING 0.05f  // lower = more smoothing
#define LIDAR_INV_SMOOTHING (1 - LIDAR_SMOOTHING)
#define LIDAR_CHECK_ANGLE 15


extern float front_distance, right_distance, left_distance;

void lidarSetup();
