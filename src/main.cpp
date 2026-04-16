#include "config.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "odom.hpp"
#include "telemetry.hpp"

// ===== Encoderインスタンス =====
Encoder enc_right(RIGHT_ENC_A, RIGHT_ENC_B);
Encoder enc_left (LEFT_ENC_A , LEFT_ENC_B );

Servo esc;

// ===== ISR =====
void isr_right_A(){
  enc_right.handleA();
}

void isr_left_A(){
  enc_left.handleA();
}

void setup() {
  Serial.begin(115200);
  /*Encode setup*/
  enc_right.begin();
  enc_left.begin();

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isr_right_A, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A ), isr_left_A , RISING);

  esc.attach(RIGHT_ESC_PIN, 1000, 2000);

  esc.writeMicroseconds(ESC_NEUTRAL);
  delay(1000);
}

void loop() {
  static unsigned long last_time = 0;
  unsigned long now = millis();

  // 速度更新
  enc_right.updateVelocity(now);
  enc_left.updateVelocity(now);

  if (now - start_time < RUN_TIME) {
    esc.writeMicroseconds(1100);
  } else {
    esc.writeMicroseconds(ESC_NEUTRAL);
  }
  
  if (now - last_time >= MEASURE_PERIOD) {
    last_time = now;

    long count_r = enc_right.getCount();
    long count_l = enc_left.getCount();

    float omega_r = enc_right.getRadPerSec();
    float omega_l = enc_left.getRadPerSec();


    Serial.print("R count: ");
    Serial.print(count_r);
    Serial.print("  omega: ");
    Serial.print(omega_r);

    Serial.print(" | L count: ");
    Serial.print(count_l);
    Serial.print("  omega: ");
    Serial.println(omega_l);
  }
  delay(CONTROL_PERIOD);
}
