#include <Arduino.h>
#include <debug.h>
#include <imu.h>
#include <lidar.h>
#include <math.h>
#include <navigation.h>
#include <pid.h>
#include <slave.h>
#include <timer.h>
#include <vector>

#include "nav_parameters.h"

#define BLOCK_MODE

PID servoPid(4.3f, 0.16f, 0.25f);
Imu imu;
Timer nav_timer(20);

#define ENCODERS_TO_MM 1.394f
Vector position = Vector();
void updatePosition(Vector* pos, float angle, int encoders)
{
    pos->x += encoders * cos(angle) * ENCODERS_TO_MM;
    pos->y += encoders * sin(angle) * ENCODERS_TO_MM;
    debug_position(*pos);
}

enum NavState {
    LidarStart, // navigate using lidars
    BlockSearch, // navigate through the blocks and save the path
    TurnEnd, // calculations on turn end
    TurnAround, // turn around at the end of the block fase
    PathCalc, // calculate the path
    PathFollow, // follow the path
    RoundEnd,
    SquareStart,
    SquareNav
};

// nav state
Axis cur_axis(position, 0, 0, 0);
#ifdef BLOCK_MODE
NavState current_state = NavState::LidarStart;
#else
NavState current_state = NavState::SquareStart;
#endif
int turn_count = 0;
int counter_clock = 1; // -1 for clockwise

const std::vector<Axis> counterClockAxes = {
    Axis(position, 0, 1, 0, 500),
    Axis(position, 1, 1, 1000, 1500),
    Axis(position, 2, 1, 2000, -500),
    Axis(position, 3, 1, -1000, 500),
};

const std::vector<Axis> clockAxes = {
    Axis(position, 0, -1, 0, 500),
    Axis(position, 1, -1, 1000, -1500),
    Axis(position, 2, -1, -2000, -500),
    Axis(position, 3, -1, -1000, -500),
};

// path following vars
int axisIndex = 0;
std::vector<Axis> centerAxes;
std::vector<Axis> path;

// blocks
bool lastBlockRed = 0; // true fir red, false for true
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

        case NavState::LidarStart: {
            // left turn
            if (left_distance > TURN_TRIGGER_DISTANCE) {
                debug_msg("locking in left turns");
                counter_clock = 1;
                centerAxes = counterClockAxes;
                current_state = NavState::BlockSearch;
            }
            // right turn
            else if (right_distance > TURN_TRIGGER_DISTANCE) {
                debug_msg("locking in right turns");
                counter_clock = -1;
                centerAxes = clockAxes;
                current_state = NavState::BlockSearch;
            }
            break;
        }
        case NavState::BlockSearch: {
            Axis center_axis = centerAxes[turn_count % 4];

            if (red_block.in_scene || green_block.in_scene) {
                cur_axis.resetDistanceTraveled(position);
                lastBlockRed = red_block.in_scene;
            }

            // if we see a block, we have to change the axis and save the prevoius one
            if (not redSeen && red_block.in_scene) {
                debug_msg("red block seen, turn: %i, sign: %i", turn_count, -center_axis.dir);
                redSeen = true;
                greenSeen = false;
                cur_axis.setEnd(position);
                path.push_back(cur_axis);
                cur_axis = Axis(
                    position, turn_count, counter_clock, center_axis.target - BLOCK_FIXED_OFFSET * center_axis.dir);
            } else if (not greenSeen && green_block.in_scene) {
                debug_msg("green block seen, turn: %i, sign: %i", turn_count, center_axis.dir);
                redSeen = false;
                greenSeen = true;
                cur_axis.setEnd(position);
                path.push_back(cur_axis);
                cur_axis = Axis(
                    position, turn_count, counter_clock, center_axis.target + BLOCK_FIXED_OFFSET * center_axis.dir);

                // same thing here, we reset the axis and save the previous one
            } else if ((greenSeen || redSeen) && cur_axis.distanceTraveled(position) > BLOCK_AFTER_DISTANCE) {
                redSeen = false;
                greenSeen = false;
                cur_axis.setEnd(position);
                path.push_back(cur_axis);
                cur_axis = center_axis;
            } else if (not greenSeen && not redSeen) {
                cur_axis = center_axis;
            }

            while (turn_count == 12 && position.x > 0) {
                motorSpeed(0);
                delay(100);
            }

            if (center_axis.finished(position)) {
                current_state = NavState::TurnEnd;
            }
            break;
        }
        case NavState::TurnEnd: {
            debug_msg("ending turn on x: %f, y: %f, turn: %i", position.x, position.y, turn_count);
            cur_axis.print();

            turn_count++;
            cur_axis.setEnd(position);
            path.push_back(cur_axis);

            redSeen = false;
            greenSeen = false;

            current_state = NavState::BlockSearch;

            // we need to reverse everything
            if (turn_count == 9 && lastBlockRed) {
                current_state = NavState::TurnAround;
            }
            // if (turn_count == 5) {
            //     current_state = NavState::PathCalc;
            // }

            break;
        }
        case NavState::TurnAround: {
            debug_msg("Reached last round turn, and last block was red, turning around");

            turn_count = 0;
            if (counter_clock == 1) {
                centerAxes = clockAxes;
            } else {
                centerAxes = counterClockAxes;
            }
            counter_clock *= -1;

            break;
        }
        case NavState::PathCalc: {
            debug_msg("calculating path to follow, starting with axis:");
            // delete all data from the first stretch, we get it from the second pass
            while (path[0].turn == 0) {
                path.erase(path.begin());
            }
            cur_axis = path[0];
            cur_axis.print();
            current_state = NavState::PathFollow;
            motorSpeed(210);
            debug_msg("------------------- FINAL PATH ----------------------");
            for (Axis a : path) {
                a.print();
            }
            debug_msg("------------------- PATH END ------------------------");
            break;
        }
        case NavState::PathFollow: {
            if (cur_axis.finished(position)) {
                int prev_follow_y = path.at(axisIndex).follow_y;
                axisIndex = (axisIndex + 1) % path.size();

                cur_axis.print();
                cur_axis = path.at(axisIndex);
                if (prev_follow_y != cur_axis.follow_y) {
                    debug_msg("Axis changed y follow, increasing turn count...");
                    turn_count++;
                }
            }

            break;
        }
        case NavState::SquareStart: {

            break;
        }
        }

        // follow the axis
        float target_angle = cur_axis.follow(position) + (turn_count * (PI / 2) * counter_clock);

        servoPid.target = target_angle;
        debug_target_direction(servoPid.target);
        servoAngle(servoPid.update(imu.rotation));
    }
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
        auto distances = lidarInitialDistances();
        position.y = 500 - distances[0];
        position.x = 1500 - distances[2] - 150;
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

    cur_axis = Axis(position, 0, 1, 0);
}
