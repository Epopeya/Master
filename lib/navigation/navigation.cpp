#include "navigation.h"
#include "vector.h"
#ifdef ARDUINO
#include <Arduino.h>
#else
#include "../../test/simulator.h"
#endif
#include <debug.h>
#include <math.h>

Navigation::Navigation(float error_coefficient, float angle_limit, float min_after_distance, float block_offset)
    : ERROR_COEFFICIENT(error_coefficient)
    , ANGLE_LIMIT(angle_limit)
    , MIN_AFTER_DISTANCE(min_after_distance)
    , BLOCK_OFFSET(block_offset)
{
}

float Navigation::angleToAxis(float from, float to)
{
    float angle = (to - from) * ERROR_COEFFICIENT;
    return constrain(angle, -ANGLE_LIMIT, ANGLE_LIMIT);
}

float Navigation::cameraOffset(vector2_t* position, int zone, bool green_block, bool red_block)
{
    float pos = (turn_count % 2) ? position->y : position->x;
    if (green_block && camera_offset == 0) {
        if (camera_offset == 0) {
            debug_msg("green block");
        }
        last_block = pos;
        return BLOCK_OFFSET; // left
    } else if (red_block && camera_offset == 0) {
        if (camera_offset == 0) {
            debug_msg("red block");
        }
        last_block = pos;
        return -BLOCK_OFFSET; // right
    }

    if (abs(last_block - pos) > MIN_AFTER_DISTANCE) {
        return 0;
    }
    return camera_offset;
}

void Navigation::update(vector2_t* position, bool green_block, bool red_block)
{
    float to = 0;
    float sign = 0;
    int zone;

    if (position->y < 500 && position->y > -500 && position->x < 500) {
        to = 0;
        sign = 1;
        zone = 0;
    } else if (position->x > 500 && position->y < 1500 && position->y > -1500) {
        to = 1000;
        sign = -1 * orientation;
        zone = 1;
    } else if (position->x > -500 && (position->y > 1500 || position->y < -1500)) {
        to = 2000 * orientation;
        sign = -1;
        zone = 2;
    } else if (position->x < -500 && (position->y > 500 || position->y < -500)) {
        to = -1000;
        sign = 1 * orientation;
        zone = 3;
    }

    camera_offset = cameraOffset(position, zone, green_block, red_block);
    to += camera_offset * sign;

    if (zone != last_zone) {
        turn_count += orientation;
        camera_offset = 0;
        debug_msg("zone change to turn: %i 🦔", turn_count);
    }

    float from = (turn_count % 2) ? position->x : position->y;
    angle = turn_count * (M_PI / 2) + (angleToAxis(from, to) * sign);
    last_zone = zone;
}
