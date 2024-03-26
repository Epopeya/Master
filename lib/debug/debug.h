#pragma once
#include <vector.h>

void debug_init();
void debug_msg(const char *format, ...);
void debug_target_direction(float angle);
void debug_current_direction(float angle);
void debug_battery(float voltage);
void debug_position(vector2_t pos);
void debug_waypoints(vector2_t waypoints[], size_t waypoints_len);
