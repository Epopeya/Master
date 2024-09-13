#include <Arduino.h>
#include <algorithm>
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

PID servoPid(1.7f, 0.76f, 0.15f);
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
    BlockStart, // navigate using lidars
    BlockSearch, // navigate through the blocks and save the path
    TurnEnd, // calculations on turn end
    TurnAroundStart, // reset all variables to prepare for turning
    TurnAround1, // turn around at the end of the block fase
    TurnAround2, // second phase, in reverse
    PathStart, // calculate the path
    PathFollow, // follow the path
    SquareStart,
    SquareCheck,
    SquareDoubleCheck,
    SquareFollow,

    // debug states
    CalibrateEncoders,
    Empty,
};

NavState current_state = NavState::BlockStart;

// general nav vars
const int MOTOR_SLOW = 150;
const int MOTOR_FAST = 210;

Axis cur_axis(position, 0, 0, 0);

int turn_count = 0;
int counter_clock = 1; // -1 for clockwise

// path following vars
int axisIndex = 0;
std::vector<Axis> path;

// square vars
std::vector<float> initial_distances;
unsigned long stop_until = 0;
const int SEPARATION_FROM_WALL = 250;
const int DISTANCE_TO_VISIBILITY = 100;

// block vars
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

std::vector<Axis> centerAxes = counterClockAxes;

bool lastBlockRed = false; // true fir red, false for green
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

        case NavState::BlockStart: {
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
                debug_msg("finished :)");
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
                current_state = NavState::TurnAroundStart;
            }
            // if (turn_count == 5) {
            //     current_state = NavState::PathCalc;
            // }

            break;
        }
        case NavState::TurnAroundStart: {
            debug_msg("Reached last round turn, and last block was red, turning around");

            // This makes sense if you visualize the transformation of the position on the 8
            imu.rotation -= 4 * PI * counter_clock + PI * counter_clock;
            position.x *= -1;
            position.y *= -1;

            turn_count = 0;

            // swap active center axes
            if (counter_clock == 1) {
                centerAxes = clockAxes;
            } else {
                centerAxes = counterClockAxes;
            }
            counter_clock *= -1;

            current_state = NavState::TurnAround1;
            motorSpeed(MOTOR_SLOW);
            break;
        }
        case NavState::TurnAround1: {
            debug_msg("turning around: %f", imu.rotation);
            float target_angle = counter_clock == 1 ? PI / 2 : -PI / 2;
            if (imu.rotation > target_angle + 0.2) {
                servoAngle(-45);
                return;
            } else if (imu.rotation < target_angle - 0.2) {
                servoAngle(45);
                return;
            } else {
                debug_msg("Finished turn around 1");
                motorSpeed(-MOTOR_SLOW);
                current_state = NavState::TurnAround2;
            }
            break;
        }
        case NavState::TurnAround2: {
            float target_angle = counter_clock == 1 ? 0 : 0;
            if (imu.rotation > target_angle + 0.2) {
                servoAngle(45);
                return;
            } else if (imu.rotation < target_angle - 0.2) {
                servoAngle(-45);
                return;
            } else {
                motorSpeed(MOTOR_SLOW);
                current_state = NavState::BlockSearch;
            }
            break;
        }
        case NavState::PathStart: {
            debug_msg("calculating path to follow, starting with axis:");
            // delete all data from the first stretch, we get it from the second pass
            while (path[0].turn == 0) {
                path.erase(path.begin());
            }
            cur_axis = path[0];
            cur_axis.print();
            current_state = NavState::PathFollow;
            motorSpeed(MOTOR_FAST);
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
            cur_axis = Axis(position, 0, 1, 0, 500 + DISTANCE_TO_VISIBILITY);
            current_state = NavState::SquareCheck;
            break;
        }
        case NavState::SquareCheck: {
            if (cur_axis.finished(position) && stop_until == 0) {
                motorSpeed(0);
                stop_until = millis() + 1500;
            }
            if (stop_until != 0 && stop_until < millis()) {
                stop_until = 0;
                motorSpeed(MOTOR_FAST);
                switch (turn_count) {
                case 0: {
                    if (left_distance > 1500) {
                        turn_count++;
                        debug_msg("turning");
                        cur_axis = Axis(position, turn_count, 1, 500 + SEPARATION_FROM_WALL, 1500 + DISTANCE_TO_VISIBILITY);
                        counter_clock = 1;
                        if (initial_distances[0] + initial_distances[1] > 900) {
                            path.push_back(Axis(position, 0, counter_clock, 500 - SEPARATION_FROM_WALL, 500));
                        } else {
                            path.push_back(Axis(position, 0, counter_clock, -500 + SEPARATION_FROM_WALL, 500));
                        }
                        position.y = position.y + (-500 + initial_distances[1]);
                    } else if (right_distance > 1500) {
                        turn_count++;
                        debug_msg("turning");
                        cur_axis = Axis(position, turn_count, -1, 1000 + SEPARATION_FROM_WALL, -1550);
                        counter_clock = -1;
                        if (initial_distances[0] + initial_distances[1] > 900) {
                            path.push_back(Axis(position, 0, counter_clock, -500 + SEPARATION_FROM_WALL, 500));
                        } else {
                            path.push_back(Axis(position, 0, counter_clock, 500 - SEPARATION_FROM_WALL, 500));
                        }
                        position.y = position.y + (500 - initial_distances[0]);

                    } else {
                        cur_axis = Axis(position, 0, 1, 0, 500 + DISTANCE_TO_VISIBILITY * 3);
                        current_state = NavState::SquareDoubleCheck;
                    }
                    break;
                }
                case 1: {
                    turn_count++;
                    path.push_back(cur_axis);
                    if ((counter_clock == 1 ? left_distance : right_distance) > 1500) {
                        cur_axis = Axis(position, turn_count, counter_clock, (1500 + SEPARATION_FROM_WALL) * counter_clock, -500 - DISTANCE_TO_VISIBILITY);
                    } else {
                        cur_axis = Axis(position, turn_count, counter_clock, (2000 + SEPARATION_FROM_WALL) * counter_clock, -500 - DISTANCE_TO_VISIBILITY);
                    }
                    break;
                }
                case 2: {
                    turn_count++;
                    path.push_back(cur_axis);
                    if ((counter_clock == 1 ? left_distance : right_distance) > 1500) {
                        cur_axis = Axis(position, turn_count, counter_clock, -500 - SEPARATION_FROM_WALL, (500 - DISTANCE_TO_VISIBILITY) * counter_clock);
                    } else {
                        cur_axis = Axis(position, turn_count, counter_clock, -1000 - SEPARATION_FROM_WALL, (500 - DISTANCE_TO_VISIBILITY) * counter_clock);
                    }
                    break;
                }
                case 3: {
                    turn_count++;
                    path.push_back(cur_axis);
                    cur_axis = path[0];
                    current_state = NavState::SquareFollow;
                    break;
                }
                }
            }
            break;
        }
        case NavState::SquareDoubleCheck: {
            if (cur_axis.finished(position)) {
                debug_msg("square double check start");
                motorSpeed(0);
                stop_until = millis() + 1500;
            }
            if (stop_until != 0 && stop_until < millis()) {
                stop_until = 0;
                motorSpeed(MOTOR_FAST);
                turn_count++;
                if (left_distance > 1500) {
                    counter_clock = 1;
                    if (initial_distances[0] + initial_distances[1] > 900) {
                        debug_msg("Wall not moved");
                        path.push_back(Axis(position, 0, counter_clock, 0, 500));
                    } else {
                        debug_msg("Wall moved!!!!!!");
                        path.push_back(Axis(position, 0, counter_clock, 250, 500));
                    }
                    position.y = position.y + (-500 + initial_distances[1]);

                } else if (right_distance > 1500) {
                    counter_clock = -1;
                    if (initial_distances[0] + initial_distances[1] > 900) {
                        path.push_back(Axis(position, 0, counter_clock, 0, 500));
                    } else {
                        path.push_back(Axis(position, 0, counter_clock, 250, 500));
                    }
                    position.y = position.y + (500 - initial_distances[0]);

                } else {
                    motorSpeed(0);
                    debug_msg("🤯");
                }
                cur_axis = Axis(position, turn_count, counter_clock, 1500 - SEPARATION_FROM_WALL, (1500 + DISTANCE_TO_VISIBILITY) * counter_clock);
                current_state = NavState::SquareCheck;
            }
            break;
        }
        case NavState::SquareFollow: {
            if (cur_axis.finished(position)) {
                turn_count++;
                cur_axis = path[turn_count % 4];
            }
            break;
        }
        case NavState::CalibrateEncoders: {
            while (position.x > 3000) {
                motorSpeed(0);
                debug_msg("final position: %f", position.x);
                delay(500);
            }
            break;
        }
        case NavState::Empty: {
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
        position.x = 1500 - distances[2] - 150;
        initial_distances = distances;
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
        motorSpeed(MOTOR_SLOW);
    }

    servoPid.target = 0;

    cur_axis = Axis(position, 0, 1, 0);
}
