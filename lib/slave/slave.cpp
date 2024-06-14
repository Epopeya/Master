#include "slave.h"
#include "debug.h"
#include <Arduino.h>

HardwareSerial hs(1);
int encoders;
float battery;

block_t red_block;
block_t green_block;

// WARN: This will block until slave communicates!
void slaveSetup() {
  hs.begin(921600, SERIAL_8N1, 4, 2);
  red_block = {.in_scene = false, .x = 0, .y = 0};
  green_block = {.in_scene = false, .x = 0, .y = 0};
  battery = 0;
  encoders = 0;

  // Block until a packet is available
  while (hs.available() < 1) {
  }
}

enum SerialCommands {
  SerialMotor,
  SerialServo,
  SerialEncoder,
  SerialBattery,
  SerialBlocks
};

void motorSpeed(int speed) {
  uint8_t buf[] = {0x16,
                   SerialMotor,
                   speed & 0xff,
                   (speed >> 8) & 0xff,
                   (speed >> 16) & 0xff,
                   (speed >> 24) & 0xff};
  hs.write(buf, sizeof(buf));
}

void servoAngle(float angle) {
  uint8_t buf[] = {0x16, SerialServo, 0, 0, 0, 0};
  memcpy(buf + 2, &angle, sizeof(float));
  hs.write(buf, sizeof(buf));
}

// Returns the difference in encoders since last called
int getEncoders() {
  int enc = encoders;
  encoders = 0;
  return enc;
}

#define BATTERY_CONVERSION 390.0f
void parseBattery() {
  uint8_t buf[2] = {0};
  hs.readBytes(buf, sizeof(buf));
  battery = (buf[0] | buf[1] << 8) / BATTERY_CONVERSION;
}

void parseEncoders() {
  uint8_t enc = hs.read();
  encoders += enc;
}

void parseBlocks() {
  uint8_t buf[9] = {0};
  hs.readBytes(buf, sizeof(buf));

  int x = buf[1] | (buf[2] << 8) | (buf[3] << 16) | (buf[4] << 24);
  int y = buf[5] | (buf[6] << 8) | (buf[7] << 16) | (buf[8] << 24);
  bool in_scene = !(x < 0 && y < 0);
  debug_msg("x: %i, y: %i", x, y);

  switch (buf[0]) {
  case 0: // Green
  {
    green_block.in_scene = in_scene;
    if (in_scene) {
      green_block.x = x;
      green_block.y = y;
    }
    break;
  }
  case 1: // Red
  {
    red_block.in_scene = in_scene;
    if (in_scene) {
      red_block.x = x;
      red_block.y = y;
    }
    break;
  }
  }
}

void slaveProcessSerial() {
  while (hs.available() > 0) {
    int start = hs.read();
    if (start == 0x16) {
      int header = hs.read();
      switch (header) {
      case SerialEncoder: {
        parseEncoders();
        break;
      }
      case SerialBattery: {
        parseBattery();
        break;
      }
      case SerialBlocks: {
        parseBlocks();
        break;
      }
      }
    }
  }
}
