#pragma once

void slaveSetup();
void servoAngle(float angle);
void motorSpeed(int speed);
int getEncoders();
void slaveProcessSerial();

typedef struct {
  bool in_scene;
  int x;
  int y;
} block_t;

extern block_t red_block;
extern block_t green_block;
