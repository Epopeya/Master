#pragma once
#include <vector>

extern float front_distance, right_distance, left_distance;
void lidarSetup();
std::vector<float> lidarInitialDistances();
void lidarStart();
