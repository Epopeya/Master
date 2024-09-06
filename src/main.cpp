#include "nav_parameters.h"
#include <Arduino.h>
#include <debug.h>
#include <imu.h>
#include <lidar.h>
#include <math.h>
#include <navigation.h>
#include <pid.h>
#include <slave.h>
#include <timer.h>
#include <vector.h>
#include <vector>

PID servoPid(5.3f, 0.08f, 0.15f);
Imu imu;
Timer nav_timer(20);

#define ENCODERS_TO_MM 1.6f
Vector position = Vector();
void updatePosition(Vector* pos, float angle, int encoders)
{
    pos->x += encoders * cos(angle) * ENCODERS_TO_MM;
    pos->y += encoders * sin(angle) * ENCODERS_TO_MM;
    debug_position(*pos);
}

enum NavState {
    LidarFollow, // navigate using lidars
    TurnEnd, // calculations on turn end
    PathCalc, // calculate the path
    PathFollow // follow the path
};

// nav state
Axis cur_axis(position, 0, 0, 0);
NavState current_state = NavState::LidarFollow;
int turn_count = 0;
int counter_clock = 1; // -1 for clockwise
float turn_center = 0; // middle of the walls for the current turn
Vector last_pos;

// path following vars
int axisIndex = 0;
std::vector<Axis> path;

// blocks
bool redSeen = false; // if a red block is on the current turn
bool greenSeen = false; // same for green

void loop()
{
    slaveProcessSerial();
    if (imu.update()) {
        debug_current_direction(imu.rotation);
    }

    if (nav_timer.primed()) {
        updatePosition(&position, imu.rotation, getEncoders());

        switch (current_state) {

        case NavState::LidarFollow: {
            // this only runs when a block is seen for the first time
            if (not redSeen && red_block.in_scene) {
                debug_msg("red block seen");
                redSeen = true;
                greenSeen = false;
                cur_axis.setEnd(position);
                path.push_back(cur_axis);
                cur_axis = Axis(position, turn_count, counter_clock, turn_center + BLOCK_FIXED_OFFSET * cur_axis.dir);
            } else if (not greenSeen && green_block.in_scene) {
                debug_msg("green block seen");
                redSeen = false;
                greenSeen = true;
                cur_axis.setEnd(position);
                path.push_back(cur_axis);
                cur_axis = Axis(position, turn_count, counter_clock, turn_center - BLOCK_FIXED_OFFSET * cur_axis.dir);
            }

            // this has to be tested in real life
            /* // reset the distance to travel while seeing a block */
            /* if (red_block.in_scene || green_block.in_scene) { */
            /*     cur_axis.resetDistanceTraveled(position); */
            /* } */

            /* // return to the center after a meter of no blocks */
            /* if ((not red_block.in_scene && not green_block.in_scene) */
            /*     && cur_axis.distanceTraveled(position) > BLOCK_AFTER_DISTANCE) { */
            /*     debug_msg("No block seen after a meter, returning to the center"); */
            /*     cur_axis.setEnd(position); */
            /*     path.push_back(cur_axis); */
            /*     cur_axis = Axis(position, turn_count, counter_clock, turn_center); */
            /* } */

            // TODO: once counter_clock is set, only turn in that direction
            // using manhatten distance here for simplicity
            float dist = abs(last_pos.x - position.x) + abs(last_pos.y - position.y);
            // left turn
            if (dist > TURN_TIMEOUT_DISTANCE && left_distance > TURN_TRIGGER_DISTANCE) {
                counter_clock = 1;
                current_state = NavState::TurnEnd;
            }
            // right turn
            else if (dist > TURN_TIMEOUT_DISTANCE && right_distance > TURN_TRIGGER_DISTANCE) {
                counter_clock = -1;
                current_state = NavState::TurnEnd;
            }
            break;
        }
        case NavState::TurnEnd: {

            debug_msg("ending turn on x: %f, y: %f, turn: %i", position.x, position.y, turn_count);

            // are we facing a positive or negative direction? TODO: use axis angle or smnt
            float forward_sign = 0.0f;
            if (counter_clock > 0) {
                float values[] = { 1, 1, -1, -1 };
                forward_sign = values[turn_count % 4];
            } else {
                float values[] = { 1, -1, -1, 1 };
                forward_sign = values[turn_count % 4];
            }
            //  forward_sign = sign(turn_count % 2 ? sin(cur_axis.angle_offset) : cos(cur_axis.angle_offset));
            turn_center = turn_count % 2 ? position.y : position.x;

            turn_center = (turn_center + (turn_center + forward_sign * front_distance)) / 2.;
            // turn_center = (turn_center + forward_sign * 500.);

            debug_msg("turn_pos: %f, forward_sign: %f, turn_centr: %f", turn_count % 2 ? position.x : position.y, forward_sign, turn_center);

            turn_count++;
            cur_axis.setEnd(position);
            path.push_back(cur_axis);
            cur_axis = Axis(position, turn_count, counter_clock, turn_center);

            last_pos = position;
            redSeen = false;
            greenSeen = false;

            current_state = NavState::LidarFollow;
            if (turn_count == 5) {
                current_state = NavState::PathCalc;
            }

            break;
        }
        case NavState::PathCalc: {
            imu.rotation -= counter_clock * (2. * PI); // there is probably a better solution... same TODO as below

            // delete all data from the first stretch, we get it from the second pass
            while (path[0].turn == 0) {
                path.erase(path.begin());
            }
            cur_axis = path[0];
            cur_axis.print();
            current_state = NavState::PathFollow;

            break;
        }
        case NavState::PathFollow: {
            if (cur_axis.finished(position)) {
                axisIndex = (axisIndex + 1) % path.size();

                // TODO: this is really bad... please fix
                if (axisIndex == 0) {
                    imu.rotation -= counter_clock * (2. * PI);
                }

                cur_axis.print();
                cur_axis = path.at(axisIndex);
            }

            break;
        }
        }

        // follow the axis
        float target_angle = cur_axis.follow(position);

        // wall failsafe, maybe too much...
        if (left_distance < FAILSAFE_TRIGGER_DISTANCE) {
            debug_msg("Getting too close to left, moving away...");
            target_angle -= FAILSAFE_CORRECTION_FACTOR;
        } else if (right_distance < FAILSAFE_TRIGGER_DISTANCE) {
            debug_msg("Getting too close to right, moving away...");
            target_angle += FAILSAFE_CORRECTION_FACTOR;
        }

        servoPid.target = target_angle;
        debug_target_direction(servoPid.target);
        servoAngle(servoPid.update(imu.rotation));
    }
}

float start_distance_y;
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
        Vector start_distances = lidarInitialPosition();
        position.x = 1500 - start_distances.x - 150; // for lidar offset
        position.y = 500 - start_distances.y;
        start_distance_y = start_distances.y;
        lidarStart();
    }
    debug_msg("Setup completed");

    // wait for button
    digitalWrite(26, HIGH);
    while (digitalRead(33)) { }
    digitalWrite(26, LOW);
    delay(500);

    if (battery < 4) {
        motorSpeed(0);
    } else {
        motorSpeed(150);
    }

    servoPid.target = 0;
    last_pos = position;

    // to correctly turn on the first curve
    last_pos.x -= 1000;

    cur_axis = Axis(position, 0, 0, position.y);
}
