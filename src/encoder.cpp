#include "encoder.hpp"

Encoder::Encoder(uint8_t pin_a, uint8_t pin_b)
    :pin_a_(pin_a), pin_b_(pin_b){}

void Encoder::begin(){
    pinMode(pin_a_, INPUT_PULLUP);
    pinMode(pin_b_, INPUT_PULLUP);
    prev_time_ms = millis();
}

void Encoder::handleA(){
    bool phaseA  = digitalRead(pin_a_);
    bool phaseB  = digitalRead(pin_b_);

    if(phaseA  == phaseB )
        count++;
    else
        count--;
}

void Encoder::handleB(){
    bool phaseA  = digitalRead(pin_a_);
    bool phaseB  = digitalRead(pin_b_);

    if(phaseA  != phaseB )
        count++;
    else
        count--;
}

long Encoder::getCount() const{
    // 割り込み処理の停止（安全上)
    noInterrupts();
    long count_copy = count;
    interrupts();
    return count_copy;
}

void Encoder::resetCount(){
    noInterrupts();
    count = 0;
    interrupts();

    prev_count   = 0;
    prev_time_ms = millis();
    rad_per_sec  = 0.0f;
}

void Encoder::updateVelocity(unsigned long now_ms){
    unsigned long dt_ms = now_ms - prev_time_ms;

    if(dt_ms < MEASURE_PERIOD){
        return;
    }

    long current_count = getCount();
    long delta_count   = current_count - prev_count;

    float rev  = static_cast<float>(delta_count) / PULSE_PER_REV;
    float dt_s = static_cast<float>      (dt_ms) / 1000.0f;
    
    rad_per_sec = rev * 2.0f * PI / dt_s;
    prev_count = current_count;
    prev_time_ms = now_ms;
}

float Encoder::getRadPerSec() const{
    return rad_per_sec;
}