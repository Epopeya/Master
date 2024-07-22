#include "simulator.h"
#include "vector.h"
#include <math.h>

vector2_t position = { .x = 0, .y = 0 };
float rotation = 0;
int simulationSpeed = 0;

void simulatorBegin(vector2_t pos, float rot)
{
    position = pos;
    rotation = rot;
}

void simulatorTick()
{
    position.x += simulationSpeed * cos(rotation);
    position.y += simulationSpeed * sin(rotation);
}

void motorSpeed(int speed)
{
    simulationSpeed = speed;
}

void servoAngle(float angle)
{
    rotation = angle;
}
