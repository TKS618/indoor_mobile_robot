#include "config.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "odom.hpp"
#include "telemetry.hpp"

#include <micro_ros_platformio.h>

unsigned long start_time = 0;
constexpr bool ENABLE_SERIAL_DEBUG = false;

// ===== Encoderインスタンス =====
Encoder enc_right(RIGHT_ENC_A, RIGHT_ENC_B, RIGHT_ENCODER_INVERT);
Encoder enc_left (LEFT_ENC_A , LEFT_ENC_B, LEFT_ENCODER_INVERT);

// Motor motor_right(RIGHT_ESC_PIN, -1, -1, RIGHT_ESC_SIGN);
// Motor motor_left (LEFT_ESC_PIN , -1, -1, LEFT_ESC_SIGN);

Motor motor_right(RIGHT_PIN_1, RIGHT_PIN_2, RIGHT_ESC_SIGN, KF_RIGHT);
Motor motor_left (LEFT_PIN_1, LEFT_PIN_2, LEFT_ESC_SIGN, KF_LEFT);


Odometry odom;
Telemetry telemetry;

// ===== ISR =====
void isr_right_A(){
  enc_right.handleA();
}

void isr_right_B(){
  enc_right.handleB();
}

void isr_left_A(){
  enc_left.handleA();
}

void isr_left_B(){
  enc_left.handleB();
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  /*Encode setup*/
  enc_right.begin();
  enc_left.begin();

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isr_right_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), isr_right_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A ), isr_left_A , CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B ), isr_left_B , CHANGE);

  /*Motor setup*/
  analogWriteResolution(PWM_BIT);
  analogWriteFrequency(RIGHT_PIN_1, PWM_FREQ_HZ);
  analogWriteFrequency(RIGHT_PIN_2, PWM_FREQ_HZ);
  analogWriteFrequency(LEFT_PIN_1, PWM_FREQ_HZ);
  analogWriteFrequency(LEFT_PIN_2, PWM_FREQ_HZ);
  motor_right.begin(KP_RIGHT, KI_RIGHT, KD_RIGHT);
  motor_left.begin(KP_LEFT, KI_LEFT, KD_LEFT);

  delay(1000);
  start_time = millis();
}

void loop() {
  static unsigned long last_control_time = 0;
  static unsigned long last_print_time = 0;
  static unsigned long last_odom_pub_time = 0;
  static float omega_r = 0.0f;
  static float omega_l = 0.0f;
  static float target_r = 0.0f;
  static float target_l = 0.0f;
  static long last_print_count_r = 0;
  static long last_print_count_l = 0;

  telemetry.update();
  telemetry.updateCmdVelTimeout();
  
  unsigned long now = millis();
  if (last_control_time == 0) {
    last_control_time = now;
  }
  if (last_print_time == 0) {
    last_print_time = now;
  }

  // 速度更新
  enc_right.updateVelocity(now);
  enc_left.updateVelocity(now);
  if (now - start_time > CONTROL_PERIOD) {
    if (now - last_control_time >= CONTROL_PERIOD) {
      float dt = static_cast<float>(now - last_control_time) / 1000.0f;
      last_control_time = now;

      omega_r = enc_right.getRadPerSec();
      omega_l = enc_left.getRadPerSec();

      target_r = telemetry.getTargetRightRadPerSec();
      target_l = telemetry.getTargetLeftRadPerSec();

      motor_right.setTargetRadPerSec(target_r);
      motor_left.setTargetRadPerSec(target_l);

      motor_right.update(omega_r, dt);
      motor_left.update(omega_l, dt);
      odom.update(omega_r, omega_l, dt);
    }
  } 
  
  if(now -last_odom_pub_time >= ODOM_PUBLISH_PERIOD){
    last_odom_pub_time = now;
    telemetry.publishOdom(odom.getX(), odom.getY(), odom.getTheta(), omega_r, omega_l);
  }
  
  if (ENABLE_SERIAL_DEBUG && now - last_print_time >= PRINT_PERIOD) {
    last_print_time = now;

    int cmd_r = motor_right.getLastCommand();
    int cmd_l = motor_left.getLastCommand();

    float control_r = motor_right.getLastControl();
    float control_l = motor_left.getLastControl();
    long count_r = enc_right.getCount();
    long count_l = enc_left.getCount();
    long delta_print_count_r = count_r - last_print_count_r;
    long delta_print_count_l = count_l - last_print_count_l;
    last_print_count_r = count_r;
    last_print_count_l = count_l;

    Serial.print(" | x: ");
    Serial.print(odom.getX(), 3);

    Serial.print(" y: ");
    Serial.print(odom.getY(), 3);

    Serial.print(" theta: ");
    Serial.print(odom.getTheta(), 3);

    Serial.print(" target R: ");
    Serial.print(target_r);

    Serial.print(" target L: ");
    Serial.print(target_l);

    Serial.print(" | R omega: ");
    Serial.print(omega_r);
    Serial.print("  dcnt: ");
    Serial.print(delta_print_count_r);
    Serial.print("  control: ");
    Serial.print(control_r);
    Serial.print("  cmd: ");
    Serial.print(cmd_r);

    Serial.print(" | L omega: ");
    Serial.print(omega_l);
    Serial.print("  dcnt: ");
    Serial.print(delta_print_count_l);
    Serial.print("  control: ");
    Serial.print(control_l);
    Serial.print("  cmd: ");
    Serial.println(cmd_l);
  }
}


// test code
// #include "config.hpp"
// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("motor test");

//   pinMode(RIGHT_PIN_1, OUTPUT);
//   pinMode(RIGHT_PIN_2, OUTPUT);
//   pinMode(LEFT_PIN_1, OUTPUT);
//   pinMode(LEFT_PIN_2, OUTPUT);

//   analogWriteFrequency(RIGHT_PIN_1, 1000);
//   analogWriteResolution(PWM_BIT);
//   analogWrite(RIGHT_PIN_1, 2000);
//   digitalWrite(RIGHT_PIN_2, LOW);

//   analogWriteFrequency(LEFT_PIN_1, 1000);
//   analogWriteFrequency(LEFT_PIN_2, 1000);
//   analogWriteResolution(PWM_BIT);
//   analogWrite(LEFT_PIN_1, 2000);
//   digitalWrite(LEFT_PIN_2, LOW);
// }

// void loop() {
// }gWriteFrequency(LEFT_PIN_2, 1000);
//   analogWriteResolution(PWM_BIT);
//   analogWrite(LEFT_PIN_1, 2000);
//   digitalWrite(LEFT_PIN_2, LOW);
// }

// void loop() {
// }

// Encoder test code
// #include <Arduino.h>
// #include "config.hpp"

// struct EncoderProbe {
//   uint8_t pin_a;
//   uint8_t pin_b;
//   volatile uint8_t state;
//   volatile long quad_count;
//   volatile uint32_t transitions;
//   volatile uint32_t invalid_transitions;
//   volatile uint32_t visits[4];
// };

// EncoderProbe probe_right = {RIGHT_ENC_A, RIGHT_ENC_B, 0, 0, 0, 0, {0, 0, 0, 0}};
// EncoderProbe probe_left  = {LEFT_ENC_A,  LEFT_ENC_B,  0, 0, 0, 0, {0, 0, 0, 0}};

// constexpr unsigned long PRINT_PERIOD_MS = 100;

// uint8_t readEncoderState(const EncoderProbe& probe) {
//   uint8_t a = digitalRead(probe.pin_a) == HIGH ? 1 : 0;
//   uint8_t b = digitalRead(probe.pin_b) == HIGH ? 1 : 0;
//   return (a << 1) | b;
// }

// int8_t quadratureDelta(uint8_t prev, uint8_t current) {
//   // Index is previous state in upper two bits and current state in lower two bits.
//   // Valid forward sequence: 00 -> 01 -> 11 -> 10 -> 00.
//   static constexpr int8_t table[16] = {
//       0,  1, -1,  0,
//      -1,  0,  0,  1,
//       1,  0,  0, -1,
//       0, -1,  1,  0,
//   };
//   return table[(prev << 2) | current];
// }

// void updateProbe(EncoderProbe& probe) {
//   uint8_t current = readEncoderState(probe);
//   uint8_t prev = probe.state;

//   if (current == prev) {
//     return;
//   }

//   int8_t delta = quadratureDelta(prev, current);
//   probe.state = current;
//   probe.transitions++;
//   probe.visits[current]++;

//   if (delta == 0) {
//     probe.invalid_transitions++;
//     return;
//   }

//   probe.quad_count += delta;
// }

// void isrRightA() { updateProbe(probe_right); }
// void isrRightB() { updateProbe(probe_right); }
// void isrLeftA()  { updateProbe(probe_left); }
// void isrLeftB()  { updateProbe(probe_left); }

// void printStateBits(uint8_t state) {
//   Serial.print((state >> 1) & 1);
//   Serial.print(',');
//   Serial.print(state & 1);
// }

// void printProbe(const char* name, const EncoderProbe& probe) {
//   noInterrupts();
//   uint8_t state = probe.state;
//   long quad_count = probe.quad_count;
//   uint32_t transitions = probe.transitions;
//   uint32_t invalid_transitions = probe.invalid_transitions;
//   uint32_t visits_00 = probe.visits[0];
//   uint32_t visits_01 = probe.visits[1];
//   uint32_t visits_10 = probe.visits[2];
//   uint32_t visits_11 = probe.visits[3];
//   interrupts();

//   Serial.print(name);
//   Serial.print(" A,B=");
//   printStateBits(state);
//   Serial.print(" state=");
//   Serial.print(state, BIN);
//   Serial.print(" quad=");
//   Serial.print(quad_count);
//   Serial.print(" transitions=");
//   Serial.print(transitions);
//   Serial.print(" invalid=");
//   Serial.print(invalid_transitions);
//   Serial.print(" visits[00,01,10,11]=");
//   Serial.print(visits_00);
//   Serial.print(',');
//   Serial.print(visits_01);
//   Serial.print(',');
//   Serial.print(visits_10);
//   Serial.print(',');
//   Serial.print(visits_11);
// }

// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   pinMode(RIGHT_ENC_A, INPUT_PULLUP);
//   pinMode(RIGHT_ENC_B, INPUT_PULLUP);
//   pinMode(LEFT_ENC_A, INPUT_PULLUP);
//   pinMode(LEFT_ENC_B, INPUT_PULLUP);

//   probe_right.state = readEncoderState(probe_right);
//   probe_left.state = readEncoderState(probe_left);
//   probe_right.visits[probe_right.state] = 1;
//   probe_left.visits[probe_left.state] = 1;

//   attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isrRightA, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), isrRightB, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), isrLeftA, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), isrLeftB, CHANGE);

//   Serial.println("Encoder A/B phase probe");
//   Serial.println("Rotate wheels slowly first. If B phase is not read, visits will miss states with B=1.");
//   Serial.println("Forward sign here assumes sequence 00 -> 01 -> 11 -> 10 -> 00.");
// }

// void loop() {
//   static unsigned long last_print_ms = 0;
//   unsigned long now = millis();

//   if (now - last_print_ms < PRINT_PERIOD_MS) {
//     return;
//   }
//   last_print_ms = now;

//   Serial.print("t=");
//   Serial.print(now);
//   Serial.print("ms | ");
//   printProbe("R", probe_right);
//   Serial.print(" | ");
//   printProbe("L", probe_left);
//   Serial.println();
// }
