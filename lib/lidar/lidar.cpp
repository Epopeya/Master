#include "lidar.h"
#include "vector.h"
#include <RPLidar.h>
#include <debug.h>
#include <imu.h>
#include <navigation.h>

#define RPLIDAR_MOTOR 5
#define SMOOTHING 0.01f // lower = more smoothing
#define INV_SMOOTHING (1 - SMOOTHING)
#define CHECK_ANGLE 10

HardwareSerial lidar_serial(2);
RPLidar lidar;
TaskHandle_t lidar_task;
float front_distance, right_distance, left_distance = 0.0;
extern Imu imu;
extern int turn_count;

void lidarTask(void* pvParameters)
{
    for (;;) {
        vTaskDelay(1);
        if (IS_OK(lidar.waitPoint())) {
            RPLidarMeasurement point = lidar.getCurrentPoint();
            float distance = point.distance; // distance value in mm unit
            float angle = point.angle; // angle value in degrees

            if (!(distance < 10.0 || distance > 3000.0)) {
                float r_angle = angle + (turn_count * (PI / 2) - imu.rotation) * (360 / (2 * PI));

                // front
                if (r_angle < CHECK_ANGLE || r_angle > 360 - CHECK_ANGLE) {
                    front_distance = SMOOTHING * front_distance + INV_SMOOTHING * distance;
                }

                // right
                else if (r_angle < 90 + CHECK_ANGLE && r_angle > 90) {
                    right_distance = SMOOTHING * right_distance + INV_SMOOTHING * distance;
                }

                // left
                else if (r_angle < 270 && r_angle > 270 - CHECK_ANGLE) {
                    left_distance = SMOOTHING * left_distance + INV_SMOOTHING * distance;
                }
            }
        }
    }
}

void lidarSetup()
{
    lidar.begin(lidar_serial);
    lidar.startScan();
    analogWrite(RPLIDAR_MOTOR, 255);
}

#define MEDIAN_ROUNDS 100
std::vector<float> lidarInitialDistances()
{
    int counter_left = 0;
    int counter_right = 0;
    int counter_front = 0;
    float dist_left = 0;
    float dist_right = 0;
    float dist_front = 0;
    while (counter_front < MEDIAN_ROUNDS || counter_left < MEDIAN_ROUNDS || counter_right < MEDIAN_ROUNDS) {
        if (IS_OK(lidar.waitPoint())) {
            RPLidarMeasurement point = lidar.getCurrentPoint();
            float distance = point.distance; // distance value in mm unit
            float angle = point.angle; // angle value in degrees

            if (!(distance < 10.0 || distance > 3000.0)) {
                if (angle < 5 || angle > 355) {
                    debug_msg("front: %f", distance);
                    counter_front++;
                    dist_front += distance;
                }
                if (angle < 95 && angle > 85) {
                    debug_msg("left: %f", distance);
                    counter_left++;
                    dist_left += distance;
                }
                if (angle < 275 && angle > 265) {
                    debug_msg("right: %f", distance);
                    counter_right++;
                    dist_right += distance;
                }
            }
        }
    }

    std::vector<float> ret(3);

    ret[0] = dist_left / counter_left;
    ret[1] = dist_right / counter_right;
    ret[2] = dist_front / counter_front;
    return ret;
}

void lidarStart()
{
    xTaskCreatePinnedToCore(lidarTask, "lidar", 100000, NULL, 10, &lidar_task, 0);
}
