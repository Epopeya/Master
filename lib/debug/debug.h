#pragma once

void debug_init();
void debug_msg(const char *format, ...);
void debug_target_direction(float angle);
void debug_current_direction(float angle);
void debug_battery(float voltage);
void debug_position(float x, float y);
void debug_waypoints(float waypoints[], size_t waypoints_len);
