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
Vector lidarInitialPosition()
{
    int counter_x = 0;
    int counter_y = 0;
    Vector start_distances = Vector();
    while (counter_x < MEDIAN_ROUNDS || counter_y < MEDIAN_ROUNDS) {
        if (IS_OK(lidar.waitPoint())) {
            RPLidarMeasurement point = lidar.getCurrentPoint();
            float distance = point.distance; // distance value in mm unit
            float angle = point.angle; // angle value in degrees

            if (!(distance < 10.0 || distance > 3000.0)) {
                if (angle < 5 || angle > 355) {
                    debug_msg("posX: %f", distance);
                    counter_x++;
                    start_distances.x += distance;
                }
                if (angle < 275 && angle > 265) {
                    debug_msg("posY: %f", distance);
                    counter_y++;
                    start_distances.y += distance;
                }
            }
        }
    }
    start_distances.x /= counter_x;
    start_distances.y /= counter_y;

    return start_distances;
}

void lidarStart()
{
    xTaskCreatePinnedToCore(lidarTask, "lidar", 100000, NULL, 10, &lidar_task, 0);
}
