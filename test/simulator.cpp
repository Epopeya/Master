#include "simulator.h"
#include "debug.h"
#include "vector.h"
#include <math.h>
#include <stdlib.h>

void Simulator::tick()
{
    position.x += speed * cosf(rotation);
    position.y += speed * sinf(rotation);
    distances = calcDistances();
}

float rayLineDist(Vector p0, Vector p1, Vector p2, Vector p3)
{
    Vector s1 = p1 - p0;
    Vector s2 = p3 - p2;

    float s, t;
    s = (-s1.y * (p0.x - p2.x) + s1.x * (p0.y - p2.y)) / (-s2.x * s1.y + s1.x * s2.y);
    t = (s2.x * (p0.y - p2.y) - s2.y * (p0.x - p2.x)) / (-s2.x * s1.y + s1.x * s2.y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
        Vector inter = p0 + (t * s1);
        Vector dist = inter - p0;
        // Collision detected
        return sqrt(dist.x * dist.x + dist.y * dist.y);
    }

    return 5000; // No collision
}

distances_t Simulator::calcDistances()
{
    Vector pos = Vector(position.x, position.y - 1000);
    distances_t ret;
    if (obstacles_len < 1) {
        ret.front = 0;
        ret.right = 0;
        ret.left = 0;
        return ret;
    }
    // 0 = front, 1 = left, 2 = right
    for (int i = 0; i < 3; i++) {
        float x_coeff;
        float y_coeff;
        switch (i) {
        case 0:
            x_coeff = cosf(rotation);
            y_coeff = sinf(rotation);
            break;
        case 1:
            x_coeff = cosf(rotation + PI / 2);
            y_coeff = sinf(rotation + PI / 2);
            break;
        case 2:
            x_coeff = cosf(rotation + 3 * PI / 2);
            y_coeff = sinf(rotation + 3 * PI / 2);
            break;
        }
        Vector ray_end = Vector(pos.x + (x_coeff * 5000), pos.y + (y_coeff * 5000));

        float* distances = (float*)malloc(sizeof(float) * obstacles_len * 4);
        for (int j = 0; j < obstacles_len; j++) {
            obstacle_t obs = obstacles[j];
            distances[j * 4] = rayLineDist(pos, ray_end, obs.start, Vector(obs.start.x, obs.end.y));
            distances[j * 4 + 1] = rayLineDist(pos, ray_end, Vector(obs.start.x, obs.end.y), obs.end);
            distances[j * 4 + 2] = rayLineDist(pos, ray_end, obs.end, Vector(obs.end.x, obs.start.y));
            distances[j * 4 + 3] = rayLineDist(pos, ray_end, Vector(obs.end.x, obs.start.y), obs.start);
        }
        float min_dist = 5000;
        for (int j = 0; j < obstacles_len * 4; j++) {
            if (distances[j] < min_dist) {
                min_dist = distances[j];
            }
        }
        switch (i) {
        case 0:
            ret.front = min_dist;
            break;
        case 1:
            ret.left = min_dist;
            break;
        case 2:
            ret.right = min_dist;
            break;
        };
        free(distances);
    }
    return ret;
}
