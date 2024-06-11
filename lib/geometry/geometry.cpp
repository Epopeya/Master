#include "geometry.h"
#include <math.h>

float angleBetweenPoints(vector2_t p1, vector2_t p2) {
    return atan2(p2.y - p1.y, p2.x - p1.y);
}

float distanceBetweenPoints(vector2_t p1, vector2_t p2) {
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

vector2_t addVectors(vector2_t v1, vector2_t v2) {
    return {.x = v1.x + v2.x, .y = v1.y + v2.y};
}
