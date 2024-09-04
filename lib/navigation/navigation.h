#ifndef NAVIGATION_H
#define NAVIGATION_H
#include "vector.h"

#define ERROR_COEFICENT 0.005f
#define ANGLE_CONSTRAINT PI / 3

class Axis {
public:
    Axis(vector2_t pos, int turn, int counter_clockwise, float target);

    // returns the target angle needed to follow the line
    float follow(vector2_t pos);
    float distanceTraveled(vector2_t pos);
    bool finished(vector2_t pos);

    void resetDistanceTraveled(vector2_t pos);
    void setEnd(vector2_t pos);

    void print();

    int turn = 0; // most of the other vars come from turn
    int counter_clockwise;
    bool follow_y = false;
    int dir = 1;
    float angle_offset = 0;
    float target = 0;
    float start_pos = 0;
    float end_pos = 0;
};
#endif
