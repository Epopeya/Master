#pragma once

class Imu {
    public:
        void setup();
        float rotation;
        bool update();
    private:
        float offset;
};