#pragma once

class Imu {
public:
    void setup();
    float rotation;
    bool update();
    void calibrate();
};
