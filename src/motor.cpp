#include "motor.hpp"

static float clampFloat(float x, float min_val, float max_val) {
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

Motor::Motor(int pin1, int pin2, int output_sign)
    : pin1_(pin1), pin2_(pin2), output_sign_(output_sign)
{
    pid_.kp = 0.0f;
    pid_.ki = 0.0f;
    pid_.kd = 0.0f;
    pid_.integral = 0.0f;
    pid_.prev_error = 0.0f;
}

void Motor::begin(float kp, float ki, float kd) {
    pinMode(pin1_, OUTPUT);
    pinMode(pin2_, OUTPUT);

    analogWriteResolution(PWM_BIT);
    analogWriteFrequency(pin1_, PWM_FREQ_HZ);
    analogWriteFrequency(pin2_, PWM_FREQ_HZ);

    digitalWrite(pin1_, LOW);
    digitalWrite(pin2_, LOW);

    pid_.kp = kp;
    pid_.ki = ki;
    pid_.kd = kd;

    last_control = 0.0f;
    last_command = 0;
}

void Motor::setTargetRadPerSec(float target) {
    target_rad_per_sec = clampFloat(target, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
}

float Motor::getTargetRadPerSec() const {
    return target_rad_per_sec;
}

float Motor::applyPID(float measured, float dt) {
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = target_rad_per_sec - measured;

    pid_.integral += error * dt;

    float derivative = (error - pid_.prev_error) / dt;
    pid_.prev_error = error;

    return pid_.kp * error + pid_.ki * pid_.integral + pid_.kd * derivative;
}

void Motor::outputCommand(float control) {
    last_control = control;

    control *= static_cast<float>(output_sign_);

    int pwm = static_cast<int>(control);

    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    if (pwm > 0) {
        analogWrite(pin1_, pwm);
        digitalWrite(pin2_, LOW);
    } else if (pwm < 0) {
        digitalWrite(pin1_, LOW);
        analogWrite(pin2_, -pwm);
    } else {
        digitalWrite(pin1_, LOW);
        digitalWrite(pin2_, LOW);
    }

    last_command = pwm;
}

void Motor::update(float measured_rad_per_sec, float dt) {
    float control = applyPID(measured_rad_per_sec, dt);
    outputCommand(control);
}

void Motor::stop() {
    target_rad_per_sec = 0.0f;
    pid_.integral = 0.0f;
    pid_.prev_error = 0.0f;

    digitalWrite(pin1_, LOW);
    digitalWrite(pin2_, LOW);

    last_control = 0.0f;
    last_command = 0;
}

float Motor::getLastControl() const {
    return last_control;
}

int Motor::getLastCommand() const {
    return last_command;
}