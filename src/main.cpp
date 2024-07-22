#include "nav_parameters.h"
#include "vector.h"
#include <Arduino.h>
#include <debug.h>
#include <geometry.h>
#include <imu.h>
#include <lidar.h>
#include <math.h>
#include <navigation.h>
#include <pid.h>
#include <slave.h>
#include <timer.h>

PID servoPid(1.3f, 0.08f, 0.15f);
Imu imu;
Timer nav_timer(20);

#define ENCODERS_TO_MM 1.6f
vector2_t position = { .x = 0, .y = 0 };
void updatePosition(vector2_t* pos, float angle, int encoders)
{
    pos->x += encoders * cos(angle) * ENCODERS_TO_MM;
    pos->y += encoders * sin(angle) * ENCODERS_TO_MM;
    debug_position(*pos);
}

void setup()
{
    pinMode(33, INPUT_PULLUP);
    pinMode(26, OUTPUT);

    debug_init();
    slaveSetup();
    imu.setup();

    while (battery < 0.01f) {
        slaveProcessSerial();
    } // WARN
    debug_battery(battery);

    if (battery > 4) {
        lidarSetup();
        vector2_t start_distances = lidarInitialPosition();
        position.x = 1500 - start_distances.x - 150;
        position.y = 500 - start_distances.y;
        lidarStart();
    }
    debug_msg("Setup completed");

    // wait for button
    digitalWrite(26, HIGH);
    while (digitalRead(33)) {
    }
    digitalWrite(26, LOW);
    delay(500);

    if (battery < 4) {
        motorSpeed(0);
    } else {
        motorSpeed(300);
    }

    servoPid.target = 0;
}

Navigation nav(ERROR_COEFFICENT, ANGLE_CONSTRAINT, MIN_AFTER_DISTANCE, BLOCK_FIXED_OFFSET);

void loop()
{
    slaveProcessSerial();
    if (imu.update()) {
        debug_current_direction(imu.rotation);
    }

    if (nav_timer.primed()) {
        updatePosition(&position, imu.rotation, getEncoders());

        while (nav.turn_count >= 13) {
            motorSpeed(0);
            servoAngle(0);
            delay(100);
        }

        // Initial Turn
        if (nav.orientation == 0) {
            if (left_distance > 1500) {
                debug_msg("setting orientation to 1");
                nav.orientation = 1;
            } else if (right_distance > 1500) {
                debug_msg("setting orientation to -1");
                nav.orientation = -1;
            }
            servoPid.target = nav.angleToAxis(position.y, 0);

        } else {
            // Standard Navigation
            nav.update(&position, green_block.in_scene, red_block.in_scene);
            servoPid.target = nav.angle;
        }
        debug_target_direction(servoPid.target);
        servoAngle(servoPid.update(imu.rotation));
    }
}
