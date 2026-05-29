#include "encoder.hpp"

Encoder::Encoder(uint8_t pin_a, uint8_t pin_b, bool invert)
    : pin_a_(pin_a), pin_b_(pin_b), invert_(invert) {}

void Encoder::begin() {
    pinMode(pin_a_, INPUT_PULLUP);
    pinMode(pin_b_, INPUT_PULLUP);

    noInterrupts();
    count = 0;
    prev_state = readState();
    interrupts();

    prev_count = 0;
    prev_time_ms = millis();
    rad_per_sec = 0.0f;
}

void Encoder::handleA() {
    handleTransition();
}

void Encoder::handleB() {
    handleTransition();
}

uint8_t Encoder::readState() const {
    uint8_t a = digitalRead(pin_a_) == HIGH ? 1 : 0;
    uint8_t b = digitalRead(pin_b_) == HIGH ? 1 : 0;
    return (a << 1) | b;
}

void Encoder::handleTransition() {
    uint8_t current_state = readState();
    uint8_t previous_state = prev_state;

    if (current_state == previous_state) {
        return;
    }

    // Valid forward sequence: 00 -> 01 -> 11 -> 10 -> 00.
    static constexpr int8_t transition_table[16] = {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0,
    };

    int8_t step = transition_table[(previous_state << 2) | current_state];
    prev_state = current_state;

    if (step == 0) {
        return;
    }

    if (invert_) {
        step = -step;
    }

    count += step;
}

long Encoder::getCount() const {
    noInterrupts();
    long count_copy = count;
    interrupts();
    return count_copy;
}

void Encoder::resetCount() {
    noInterrupts();
    count = 0;
    prev_state = readState();
    interrupts();

    prev_count = 0;
    prev_time_ms = millis();
    rad_per_sec = 0.0f;
}

void Encoder::updateVelocity(unsigned long now_ms) {
    unsigned long dt_ms = now_ms - prev_time_ms;

    if (dt_ms < MEASURE_PERIOD) {
        return;
    }

    long current_count = getCount();
    long delta_count = current_count - prev_count;

    float rev = static_cast<float>(delta_count) / (PULSE_PER_REV * ENCODER_COUNT_MULTIPLIER);
    float dt_s = static_cast<float>(dt_ms) / 1000.0f;

    rad_per_sec = rev * 2.0f * PI / dt_s;

    prev_count = current_count;
    prev_time_ms = now_ms;
}

float Encoder::getRadPerSec() const {
    return rad_per_sec;
}
