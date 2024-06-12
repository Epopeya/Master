#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vector.h"

typedef enum {
  Message,
  TargetDirection,
  CurrentDirection,
  Battery,
  Position,
  Route,
  Lidar,
  MapFlip
} DebugHeader;

HardwareSerial hs_debug(0);

void debug_init() {
  hs_debug.begin(112500, SERIAL_8N1, 23, 19);
  //while (hs_debug.available() < 1) {}
}

void debug_msg(const char *format, ...) {
  char buffer[UINT8_MAX];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, UINT8_MAX, format, args);
  uint8_t msg_len = (uint8_t)strlen(buffer);
  hs_debug.write(Message);
  hs_debug.write(msg_len);
  hs_debug.write(buffer, msg_len);
}

void debug_target_direction(float angle) {
  hs_debug.write(TargetDirection);
  hs_debug.write((uint8_t *)&angle, sizeof(float));
}

void debug_current_direction(float angle) {
  hs_debug.write(CurrentDirection);
  hs_debug.write((uint8_t *)&angle, sizeof(float));
}

void debug_battery(float voltage) {
  hs_debug.write(Battery);
  hs_debug.write((uint8_t *)&voltage, sizeof(float));
}

void debug_position(vector2_t pos) {
  hs_debug.write(Position);
  hs_debug.write((uint8_t *)&pos.x, sizeof(float));
  hs_debug.write((uint8_t *)&pos.y, sizeof(float));
}

void debug_waypoints(vector2_t waypoints[], size_t waypoints_len) {
  hs_debug.write(Route);
  hs_debug.write((uint8_t)waypoints_len);
  for (int i = 0; i < waypoints_len; i++) {
    hs_debug.write((uint8_t *)&waypoints[i].x, sizeof(float));
    hs_debug.write((uint8_t *)&waypoints[i].y, sizeof(float));
  }
}

void debug_lidar(vector2_t pos) {
  hs_debug.write(Lidar);
  hs_debug.write((uint8_t *)&pos.x, sizeof(float));
  hs_debug.write((uint8_t *)&pos.y, sizeof(float));
}

void debug_map_flip(bool flipped) {
  hs_debug.write(MapFlip);
  hs_debug.write((uint8_t *)&flipped, sizeof(bool));
}