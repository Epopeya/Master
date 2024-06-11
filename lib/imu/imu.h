#pragma once

class Imu {
    public:
        int setup();
        float rotation;
        bool update();
};