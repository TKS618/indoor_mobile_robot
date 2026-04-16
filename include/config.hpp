#pragma once
#include <Arduino.h>
#include <Servo.h>

constexpr unsigned long CONTROL_PERIOD = 10; //制御周期
constexpr unsigned long RUN_TIME = 20000;      // 20秒

// ===== driver type =====
enum class MotorDriverType {
    QUICRUN,
    TB6612
};

constexpr MotorDriverType MOTOR_DRIVER = MotorDriverType::QUICRUN;

/*Encoder*/
// 右輪encoderのA, B相のピン番号
constexpr int RIGHT_ENC_A = 23;
constexpr int RIGHT_ENC_B = 22;
// 左輪encoderのA, B相のピン番号
constexpr int LEFT_ENC_A  = 41;
constexpr int LEFT_ENC_B  = 40;

constexpr float PULSE_PER_REV = 1646.2; //車輪PPR

constexpr unsigned long MEASURE_PERIOD = 100;                            //エンコーダ計測時間

/*Odometry*/


/*Motor*/
// 右輪motorのESCピン番号
constexpr int RIGHT_ESC_PIN = 21;
// 左輪motorのESCピン番号
constexpr int LEFT_ESC_PIN = 39;

// Motor回転方向
constexpr bool RIGHT_ENCODER_INVERT = false;
constexpr bool LEFT_ENCODER_INVERT  = true;

constexpr int RIGHT_ESC_SIGN = -1;  // 前方で1500以下
constexpr int LEFT_ESC_SIGN  = +1;  // 前方で1500以上

// quicrunでのmotor制御
constexpr int ESC_MIN = 1000;
constexpr int ESC_NEUTRAL = 1500;
constexpr int ESC_MAX = 2000;

// ===== PID =====
constexpr float KP_LEFT  = 200.0f;
constexpr float KI_LEFT  = 0.0f;
constexpr float KD_LEFT  = 0.0f;
constexpr float KP_RIGHT = 200.0f;
constexpr float KI_RIGHT = 0.0f;
constexpr float KD_RIGHT = 0.0f;

// 目標角速度上限
constexpr float MAX_WHEEL_RAD_S = 10.0f;

/*Telemetry*/