#pragma once
#include "vector.h"

extern float front_distance, right_distance, left_distance;
void lidarSetup();
vector2_t lidarInitialPosition();
void lidarStart();
