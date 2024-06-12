#include "slave.h"
#include <Arduino.h>

HardwareSerial hs(1);
int encoders = 0;
float battery = 0;

// WARN: This will block until slave communicates!
void slaveSetup() {
  hs.begin(921600, SERIAL_8N1, 4, 2);
  // Block until a packet is available
  while(hs.available() < 1) {}
}

enum SerialCommands {
  SerialMotor,
  SerialServo,
  SerialEncoder,
  SerialBattery,
  SerialBlocks
};

void motorSpeed(int speed) {
  uint8_t buf[] = {
    0x16,
    SerialMotor,
    speed & 0xff,
    (speed >> 8) & 0xff,
    (speed >> 16) & 0xff,
    (speed >> 24) & 0xff
  };
  hs.write(buf, sizeof(buf));
}

void servoAngle(float angle) {
  uint8_t buf[] = { 0x16, SerialServo, 0, 0, 0, 0 };
  memcpy(buf + 2, &angle, sizeof(float));
  hs.write(buf, sizeof(buf));
}

int getEncoders() {
  int enc = encoders;
  encoders = 0;
  return enc;
}

#define BATTERY_CONVERSION 390.0f

void slaveProcessSerial() {
  while (hs.available() > 0) {
    int start = hs.read();
    if (start == 0x16) {
      int header = hs.read();
      switch (header) {
        case SerialEncoder:
        {
          uint8_t enc = hs.read();
          encoders += enc;
          break;
        }
        case SerialBattery:
        {
          uint8_t buf[2] = { 0 };
          hs.readBytes(buf, sizeof(buf));
          battery = (buf[0] | buf[1] << 8) / BATTERY_CONVERSION; 
          break;
        }
      }
    }
    
  }
}
