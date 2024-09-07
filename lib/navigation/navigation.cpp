#include "navigation.h"
#include "vector.h"
#ifdef ARDUINO
#include <Arduino.h>
#else
#include "../../test/simulator.h"
#endif
#include <debug.h>

int sign(float x)
{
    return x / abs(x);
}

Axis::Axis(Vector pos, int _turn, int _counter_clockwise, float _target, float _end)
    : turn(_turn)
    , counter_clockwise(_counter_clockwise)
    , follow_y(!(_turn % 2))
    // this represents the sign that you would need to add to move to the right
    // could also be represented by the sign of the positions movement direction coodinate with an inverted y axis
    , dir(sign(cos(_turn * (PI / 2.0) + (PI / 4.0) - ((PI / 2) * (counter_clockwise == 1 ? 0 : 1)))))
    , start_pos(follow_y ? pos.x : pos.y)
    , target(_target)
    , end_pos(_end)
{
    print();
}

void Axis::print()
{
    debug_msg("Axis -> follow_y: %i, turn: %i, sign: %i, target: %f, end_pos: %f", follow_y, turn,
        dir, target, end_pos);
}

float Axis::follow(Vector pos)
{
    float follow_pos = follow_y ? pos.y : pos.x;
    float angle = (target - follow_pos) * ERROR_COEFICENT;

    angle *= dir; // this only changes the sign
    angle = constrain(angle, -ANGLE_CONSTRAINT, ANGLE_CONSTRAINT);

    return angle;
}

float Axis::distanceTraveled(Vector pos)
{
    return abs(start_pos - (follow_y ? pos.x : pos.y));
}

bool Axis::finished(Vector pos)
{
    int mod_turn = turn % 4;
    float check_pos = follow_y ? pos.x : pos.y;

    // checks if we have advanced past the end point, any better way?
    if (mod_turn == 0) {
        return check_pos > end_pos;
    } else if (mod_turn == 1) {
        return (counter_clockwise > 0) ? check_pos > end_pos : check_pos < end_pos;
    } else if (mod_turn == 2) {
        return check_pos < end_pos;
    } else if (mod_turn == 3) {
        return (counter_clockwise > 0) ? check_pos < end_pos : check_pos > end_pos;
    }
    return false;
}

void Axis::resetDistanceTraveled(Vector pos)
{
    start_pos = follow_y ? pos.x : pos.y;
}

void Axis::setEnd(Vector pos)
{
    end_pos = follow_y ? pos.x : pos.y;
}
