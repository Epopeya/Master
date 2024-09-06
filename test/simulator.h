#pragma once
#include <cstdlib>
#include <math.h>
#include <stddef.h>
#include <vector.h>

// Some defines copied from Arduino.h to achieve cross-platform compatibility
#define PI 3.1415926535897932384626433832795
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
using std::abs;

typedef struct {
    float left;
    float right;
    float front;
} distances_t;

enum Color {
    ColorNone,
    ColorRed,
    ColorGreen
};

typedef struct {
    Vector position;
    Color color;
} block_t;

typedef struct {
    Vector start;
    Vector end;
} obstacle_t;

class Simulator {
public:
    Simulator(Vector position, float rotation, int speed, obstacle_t* obstacles, size_t obstacles_len, block_t* blocks, size_t blocks_len)
        : position(position)
        , rotation(rotation)
        , speed(speed)
        , obstacles(obstacles)
        , obstacles_len(obstacles_len)
        , blocks(blocks)
        , blocks_len(blocks_len)
    {
    }
    float rotation;
    int speed;
    void tick();
    distances_t distances;
    Vector position;

private:
    obstacle_t* obstacles;
    size_t obstacles_len;
    block_t* blocks;
    size_t blocks_len;
    distances_t calcDistances();
};
