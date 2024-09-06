#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

class HardwareSerial {
public:
    void begin()
    {
        output = fopen("debug_output.raw", "w");
    }
    void write(int value)
    {
        fwrite(&value, 1, 1, output);
    }
    void write(uint8_t* buf, size_t len)
    {
        fwrite(buf, 1, len, output);
    }
    void write(char* buf, size_t len)
    {
        fwrite(buf, 1, len, output);
    }

private:
    FILE* output;
};
