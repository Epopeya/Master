#pragma once
#include <stddef.h>
#include <vector.h>

void debug_init();
void debug_msg(const char* format, ...);
void debug_target_direction(float angle);
void debug_current_direction(float angle);
void debug_battery(float voltage);
void debug_position(Vector pos);
void debug_waypoints(Vector waypoints[], size_t waypoints_len);
void debug_lidar(Vector pos);
void debug_map_flip(bool flipped);
