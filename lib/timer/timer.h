#pragma once

class Timer {
    public:
    Timer(int time): time(time) {};
    int time;
    unsigned long last_time;
    bool primed();
};