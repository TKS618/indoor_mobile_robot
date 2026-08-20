#pragma once
#include <Arduino.h>
#include <Servo.h>

constexpr unsigned long CONTROL_PERIOD = 10; //制御周期
constexpr unsigned long RUN_TIME = 20000;      // 20秒

// ===== driver type =====
enum class MotorDriverType {
    QUICRUN,
    MDD3A
    
};

constexpr MotorDriverType MOTOR_DRIVER = MotorDriverType::MDD3A;

/*Encoder*/
// 右輪encoderのA, B相のピン番号
constexpr int RIGHT_ENC_A = 23;
constexpr int RIGHT_ENC_B = 22;
// 左輪encoderのA, B相のピン番号
constexpr int LEFT_ENC_A  = 41;
constexpr int LEFT_ENC_B  = 40;

constexpr float PULSE_PER_REV = 2145.2f; //車輪PPR
constexpr float ENCODER_COUNT_MULTIPLIER = 4.0f; //A/B両相CHANGEで4逓倍

constexpr unsigned long MEASURE_PERIOD = 10;                             //エンコーダ計測時間
constexpr unsigned long PRINT_PERIOD = 100;                              //シリアル表示周期

/*Odometry*/
constexpr float WHEEL_RADIUS = 0.03357f; //半径3.4cm
constexpr float WHEEL_BASE   = 0.29398f;
constexpr float WHEEL_RADIUS_INV = 1 / WHEEL_RADIUS;
constexpr float WHEEL_BASE_INV = 1 / WHEEL_BASE;
/*Motor*/
constexpr int PWM_BIT = 12;
constexpr int PWM_MAX = (1 << PWM_BIT) - 1;
constexpr float PWM_FREQ_HZ = 1000.0f;
// 右輪motorのピン番号
constexpr int RIGHT_PIN_1 = 0;//M1A
constexpr int RIGHT_PIN_2 = 1;//M1B
// 左輪motorのESCピン番号
constexpr int LEFT_PIN_1 = 2;//M2A
constexpr int LEFT_PIN_2 = 3;//M2B

// Motor回転方向
constexpr bool RIGHT_ENCODER_INVERT = false;
constexpr bool LEFT_ENCODER_INVERT  = true;

constexpr int RIGHT_ESC_SIGN = +1;
constexpr int LEFT_ESC_SIGN  = +1;


// ===== PID =====
constexpr float KP_LEFT  = 50.0f;//10.0f
constexpr float KI_LEFT  = 1.0f;
constexpr float KD_LEFT  = 1.0f;
constexpr float KP_RIGHT = 50.0f;//100.0f
constexpr float KI_RIGHT = 1.0f;
constexpr float KD_RIGHT = 1.0f;

// ===== Velocity feed-forward =====
constexpr float KF_LEFT  = 800.0f; // cmd 4000 at 5 rad/s
constexpr float KF_RIGHT = 775.0f; // cmd 4000 at 5 rad/s
constexpr float PID_INTEGRAL_LIMIT = 2.0f; // [rad] integral(error * dt)
constexpr float TARGET_STOP_EPS = 0.01f;  // [rad/s]

// 目標角速度上限
constexpr float MAX_WHEEL_RAD_S = 5.0f;

/*Telemetry*/
constexpr unsigned long ODOM_PUBLISH_PERIOD = 50;   // [ms] 20Hz
constexpr unsigned long CMD_VEL_TIMEOUT_MS  = 1000000;  // [ms] 1000sec
