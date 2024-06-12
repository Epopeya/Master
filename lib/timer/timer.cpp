#include "timer.h"
#include <Arduino.h>

bool Timer::primed() {
    unsigned long current_time = millis();
    if ((current_time - last_time) > time) {
        last_time = current_time;
        return true;
    } else {
        return false;
    }
}