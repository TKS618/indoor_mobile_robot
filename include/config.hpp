#pragma once
#include <Arduino.h>
#include <Servo.h>

constexpr unsigned long CONTROL_PERIOD = 10; //制御周期
constexpr unsigned long RUN_TIME = 20000;      // 20秒
constexpr unsigned long start_time = 0;

/*Encoder*/
// 右輪encoderのA, B相のピン番号
constexpr int RIGHT_ENC_A = 23;
constexpr int RIGHT_ENC_B = 22;
// 左輪encoderのA, B相のピン番号
constexpr int LEFT_ENC_A  = 41;
constexpr int LEFT_ENC_B  = 40;

constexpr float PULSE_PER_REV = 1646.2; //車輪PPR

constexpr unsigned long MEASURE_PERIOD = 100;                            //エンコーダ計測時間
constexpr float         RPM_COEF       = (60.0f * 1000) / MEASURE_PERIOD;

/*Odometry*/


/*Motor*/
// 右輪motorのESCピン番号
constexpr int RIGHT_ESC_PIN = 21;
// 左輪motorのESCピン番号
constexpr int LEFT_ESC_PIN = 39;

// quicrunでのmotor制御
constexpr int ESC_MIN = 1000;
constexpr int ESC_NEUTRAL = 1500;
constexpr int ESC_MAX = 2000;

/*Telemetry*/