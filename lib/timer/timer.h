#pragma once
#include <Arduino.h>

class Timer {
    public:
    Timer(int time): time(time), last_time(millis()) {};
    int time;
    bool primed();
    private:
    unsigned long last_time;
};