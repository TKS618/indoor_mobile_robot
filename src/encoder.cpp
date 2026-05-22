#include "encoder.hpp"

Encoder::Encoder(uint8_t pin_a, uint8_t pin_b, bool invert)
    : pin_a_(pin_a), pin_b_(pin_b), invert_(invert) {}

void Encoder::begin() {
    pinMode(pin_a_, INPUT_PULLUP);
    pinMode(pin_b_, INPUT_PULLUP);

    noInterrupts();
    count = 0;
    interrupts();

    prev_count = 0;
    prev_time_ms = millis();
    rad_per_sec = 0.0f;
}

void Encoder::handleA() {
    bool phaseB = digitalRead(pin_b_);

    long step;

    // A相のRISINGで呼ばれる前提
    // 方向が逆なら config.hpp の *_ENCODER_INVERT を反転する
    if (phaseB == HIGH) {
        step = -1;
    } else {
        step = 1;
    }

    if (invert_) {
        step = -step;
    }

    count += step;
}

void Encoder::handleB() {
    // A相RISINGだけを使う場合、この関数は基本使わない
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

    float rev = static_cast<float>(delta_count) / PULSE_PER_REV;
    float dt_s = static_cast<float>(dt_ms) / 1000.0f;

    rad_per_sec = rev * 2.0f * PI / dt_s;

    prev_count = current_count;
    prev_time_ms = now_ms;
}

float Encoder::getRadPerSec() const {
    return rad_per_sec;
}