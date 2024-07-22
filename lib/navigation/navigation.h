#ifndef NAVIGATION_H
#define NAVIGATION_H
#include "vector.h"
class Navigation {
public:
    Navigation(float error_coefficient, float angle_limit, float min_after_distance, float block_offset);
    void update(vector2_t* position, bool green_block, bool red_block);
    float angle;
    // Should only be used when orientation isn't known
    float angleToAxis(float from, float to);
    int turn_count = 0;
    int orientation = 0; // Whether we are turning clockwise or counterclockwise

private:
    float cameraOffset(vector2_t* position, int zone, bool green_block, bool red_block);
    float camera_offset;
    float ERROR_COEFFICIENT;
    float ANGLE_LIMIT;
    float MIN_AFTER_DISTANCE;
    float BLOCK_OFFSET;
    int last_zone;
    float last_block;
};

#endif
