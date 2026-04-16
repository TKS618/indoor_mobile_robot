#pragma once 
#include "config.hpp"

struct PID{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
};

class Motor{
public:
    Motor(int pin1, int pin2 = -1, int pin_pwm = -1, int output_sign = -1);

    void begin(float kp, float ki, float kd);
    void setTargetRadPerSec(float target);
    float getTargetRadPerSec() const;

    void update(float measured_rad_per_sec, float dt);
    void stop();


    float getLastControl() const;
    int getLastCommand() const;

private:
    float applyPID(float measured, float dt);
    void outputCommand(float control);

    int pin1_;
    int pin2_;
    int pin_pwm_;
    int output_sign_;

    Servo esc_;
    PID pid_;

    float target_rad_per_sec = 0.0f;

    float last_control = 0.0f; //PID出力
    int last_command = ESC_NEUTRAL;
};