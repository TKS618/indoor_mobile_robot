#pragma once
#include "config.hpp"

class Encoder{
public:
    Encoder(uint8_t pin_a, uint8_t pin_b);

    void begin();

    void handleA();
    void handleB();

    long getCount() const;
    void resetCount();

    void updateVelocity(unsigned long now_ms);

    float getRadPerSec() const;

private:
    uint8_t pin_a_;
    uint8_t pin_b_;

    volatile long count      = 0;
    long          prev_count = 0;
    unsigned long prev_time_ms = 0;

    float rad_per_sec = 0.0f;
};

