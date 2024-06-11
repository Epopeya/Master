#include "timer.h"
#include <Arduino.h>

bool Timer::primed() {
    unsigned long current_time = millis();
    bool result = (current_time - last_time) > time;
    if (result) {
        last_time = current_time;
        return true;
    } else {
        return false;
    }
}