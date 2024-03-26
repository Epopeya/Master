#include <Arduino.h>
#include <MPU9250.h>
#include "debug.h"

#include <RPLidar.h>

#define MPU_ADDRESS 0x68
#define MPU_CALIBRATION_ITERATIONS 1000
#define RPLIDAR_MOTOR 5

#define MILIMETERS_PER_ENCODER 3.6f
#define SERVO_MIDPOINT 93

#define DIR_KP 0.1
#define DIR_KI 0
#define DIR_KD 0.02

#define LIDAR_SMOOTHING 0.1f  // lower = more smoothing
#define LIDAR_INV_SMOOTHING (1 - LIDAR_SMOOTHING)
#define LIDAR_CHECK_ANGLE 15

#define OUTER_LENGTH 3000
#define INNER_LENGTH 1500
#define TURNING_POINT 400
#define MIN_TURN_TIME 2000
#define WAYPOINT_MIN_DISTANCE 200

// You need to create an driver instance
HardwareSerial hs(1);
HardwareSerial lidar_serial(2);
RPLidar lidar;
MPU9250 mpu;
TaskHandle_t lidar_task;

// PID variables
float error, last_error, cum_error, rate_error;

// gyro
unsigned long gyro_last_time = 0;
long gyro_dt;
float gyro_offset;

// positioning
float current_angle = 0.0f;
float posX = 0;
float posY = -750;
float dirX = 1.0;
float dirY = 0.0;

int waypoints[8] = { 0, 0,
                     0, 0,
                     0, 0,
                     0, 0 };
int waypoint_index = 0;
float target_angle = 0.0f;
unsigned long last_turn_millis = 0;

// lidar vars
float left_distance = 0.0f;
float right_distance = 0.0f;
float front_distance = 0.0f;

enum SerialCommands {
  SerialMotor,
  SerialServo,
  SerialEncoder,
  SerialBattery
};

void motorSpeed(int speed) {
  hs.write(SerialMotor);
  hs.write((uint8_t *)&speed, sizeof(int));
}

void servoAngle(int angle) {
  hs.write(SerialServo);
  hs.write((uint8_t *)&angle, sizeof(int));
}

// use absloute angle for this
void liderTask(void *pvParameters) {
  for (;;) {
    vTaskDelay(1);


    if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance;  //distance value in mm unit
      float angle = lidar.getCurrentPoint().angle;        //anglue value in degree

      if (distance < 10.0 || distance > 3000.0) {
        continue;
      }

      float r_angle = angle + (target_angle - current_angle);
      // Serial.println(r_angle);

      // front
      if (r_angle < LIDAR_CHECK_ANGLE || r_angle > 360 - LIDAR_CHECK_ANGLE) {
        front_distance = LIDAR_SMOOTHING * front_distance + LIDAR_INV_SMOOTHING * distance;
      }

      // right
      else if (r_angle < 90 + LIDAR_CHECK_ANGLE && r_angle > 90 - LIDAR_CHECK_ANGLE) {
        right_distance = LIDAR_SMOOTHING * right_distance + LIDAR_INV_SMOOTHING * distance;
      }

      // left
      else if (r_angle < 270 + LIDAR_CHECK_ANGLE && r_angle > 270 - LIDAR_CHECK_ANGLE) {
        left_distance = LIDAR_SMOOTHING * left_distance + LIDAR_INV_SMOOTHING * distance;
      }
    }
  }
}

void calibrateImu() {
  float totalAngle = 0;
  for (int i = 0; i < MPU_CALIBRATION_ITERATIONS; i++) {
    while (!mpu.update()) {}
    totalAngle += mpu.getGyroZ();
  }
  gyro_offset = totalAngle / MPU_CALIBRATION_ITERATIONS;
}

bool updateGyro() {
  if (mpu.update()) {
    long current_millis = millis();
    gyro_dt = current_millis - gyro_last_time;
    gyro_last_time = current_millis;
    float gyro_value = (mpu.getGyroZ() - gyro_offset) * 2 * PI / 360;
    current_angle += gyro_value * gyro_dt / 1000;
    return true;
  }
  return false;
}

float computeServoSpeed() {
  error = target_angle - current_angle;
  cum_error += error * gyro_dt;
  rate_error = (error - last_error) / gyro_dt;

  float output = DIR_KP * error + DIR_KI * cum_error + DIR_KD * rate_error;

  last_error = error;

  return output;
}

// calculate new direction and change waypoints
void followWaypoint() {
  // vector pointing to target
  float targetX = waypoints[waypoint_index] - posX;
  float targetY = waypoints[waypoint_index + 1] - posY;

  // shortest angle to target
  float angle_diff = atan2(targetY, targetX) - atan2(dirY, dirX);
  if (angle_diff > PI) {
    angle_diff = angle_diff - 2 * PI;
  } else if (angle_diff < -PI) {
    angle_diff = angle_diff + 2 * PI;
  }

  target_angle = angle_diff + current_angle;

  if ((targetX * targetX + targetY * targetY) < WAYPOINT_MIN_DISTANCE * WAYPOINT_MIN_DISTANCE) {
    waypoint_index = (waypoint_index + 2) % 8;
  }
}

void setup() {
  // Motor
  hs.begin(1000000, SERIAL_8N1, 4, 2);

  Wire.begin();
  mpu.setup(MPU_ADDRESS);
  calibrateImu();

  motorSpeed(10);
  servoAngle(SERVO_MIDPOINT);

  //Debugging
  Serial.begin(9600);
  debug_init();
  //gyro_last_time = millis();

  // begin the lidar
  lidar.begin(lidar_serial);
  rplidar_response_device_info_t info;
  //while (!IS_OK(lidar.getDeviceInfo(info, 100))) delay(500);
  rplidar_response_device_health_t health;
  lidar.getHealth(health);
  //Serial.println("info: " + String(health.status) +", " + String(health.error_code));
  // detected...
  lidar.startScan();

  xTaskCreatePinnedToCore(
    liderTask,
    "liderTask",
    100000,
    NULL,
    10,
    &lidar_task,
    0);

  analogWrite(RPLIDAR_MOTOR, 255);
}

int currentTurn = 0;

void loop() {
  // Receive messages on HardwareSerial 1
  if (hs.available() > 0) {
    int header = hs.read();
    switch (header) {
      // Update position
      case SerialEncoder:
        {
          uint8_t encoders = 0;
          hs.readBytes(&encoders, 1);
          encoders *= MILIMETERS_PER_ENCODER;
          posX += encoders * dirX;
          posY += encoders * dirY;
          break;
        }
      case SerialBattery:
        {
          float voltage = 0;
          hs.readBytes((uint8_t *)&voltage, sizeof(float));
          debug_battery(voltage);
        }
    }
  }

  // update direction
  if (updateGyro()) {
    dirX = cos(current_angle);
    dirY = sin(current_angle);

    servoAngle(computeServoSpeed() * 360.0f / 2 * PI + SERVO_MIDPOINT);  // convert to degrees now
  }


  if (currentTurn >= 4) {
    motorSpeed(30);
    followWaypoint();
  } else if (front_distance < TURNING_POINT && millis() - last_turn_millis > MIN_TURN_TIME) {
    if (left_distance > INNER_LENGTH) {
      target_angle += PI / 2.0;
      waypoints[currentTurn * 2] = posX;
      waypoints[currentTurn * 2 + 1] = posY;

      currentTurn++;
      last_turn_millis = millis();
    }
  }



  debug_position(posX, posY);
  debug_current_direction(current_angle);
  debug_target_direction(target_angle);
}
