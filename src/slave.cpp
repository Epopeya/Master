#include "slave.h"
#include "position.h"

HardwareSerial hs(1);

extern float battery_voltage;
extern int total_encoders;

#define VOLTAGE_SAMPLE_N 32
int voltage_samples[VOLTAGE_SAMPLE_N];
int voltage_sample_index = 0;
float total_voltage = 0.0f;
bool battery_ready = false;
#define BATTERY_CONVERSION 390

extern block_t red_block;
extern block_t green_block;

void slaveSetup() {
  hs.begin(1000000, SERIAL_8N1, 4, 2);
}

enum SerialCommands {
  SerialMotor,
  SerialServo,
  SerialEncoder,
  SerialBattery,
  SerialBlocks
};

void motorSpeed(int speed) {
  hs.write(SerialMotor);
  hs.write((uint8_t *)&speed, sizeof(int));
}

void servoAngle(int angle) {
  hs.write(SerialServo);
  hs.write((uint8_t *)&angle, sizeof(int));
}

void receiveFromSlave() {
  while (hs.available() > 0) {
    int header = hs.read();
    switch (header) {
      // Update position
      case SerialEncoder:
        {
          uint8_t encoders = 0;
          hs.readBytes(&encoders, 1);
          total_encoders += encoders;
          encoders *= MILIMETERS_PER_ENCODER;
          position.x += encoders * orientation.x;
          position.y += encoders * orientation.y;
          break;
        }
      case SerialBattery:
        {
          uint16_t voltage = 0.0f;
          hs.readBytes((uint8_t *)&voltage, sizeof(uint16_t));
          total_voltage -= voltage_samples[voltage_sample_index];
          total_voltage += voltage;
          voltage_samples[voltage_sample_index] = voltage;
          if (voltage_sample_index == VOLTAGE_SAMPLE_N - 1) battery_ready = true;
          voltage_sample_index = (voltage_sample_index + 1) % VOLTAGE_SAMPLE_N;
          battery_voltage = total_voltage / VOLTAGE_SAMPLE_N / BATTERY_CONVERSION;
          break;
        }
      case SerialBlocks:
        {
          int color = hs.read();
          int x = 0;
          int y = 0;
          hs.readBytes((uint8_t *)&x, sizeof(int));
          hs.readBytes((uint8_t *)&y, sizeof(int));
          switch (color) {
            case 0:  // Green
              {
                green_block.in_scene = false;
                if (x < 1 || y < 1) return;
                else green_block.in_scene = true;
                green_block.x = x;
                green_block.y = y;
                debug_msg("green block: x->%d y->%d", x, y);
                break;
              }
            case 1:  // Red
              {
                red_block.in_scene = false;
                if (x < 1 || y < 1) return;
                else red_block.in_scene = true;
                red_block.x = x;
                red_block.y = y;
                debug_msg("red block: x->%d y->%d", x, y);
                break;
              }
          }
          break;
        }
    }
  }
}

void waitForBattery() {
  while (!battery_ready) {
    receiveFromSlave();
  }
}
