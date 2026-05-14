#include "config.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "odom.hpp"
#include "telemetry.hpp"

unsigned long start_time = 0;

// ===== Encoderインスタンス =====
Encoder enc_right(RIGHT_ENC_A, RIGHT_ENC_B, RIGHT_ENCODER_INVERT);
Encoder enc_left (LEFT_ENC_A , LEFT_ENC_B, LEFT_ENCODER_INVERT);

// Motor motor_right(RIGHT_ESC_PIN, -1, -1, RIGHT_ESC_SIGN);
// Motor motor_left (LEFT_ESC_PIN , -1, -1, LEFT_ESC_SIGN);

Motor motor_right(RIGHT_PIN_1, RIGHT_PIN_2, RIGHT_ESC_SIGN);
Motor motor_left (LEFT_PIN_1, LEFT_PIN_2, LEFT_ESC_SIGN);


Odometry odom;
Telemetry telemetry;
// constexpr float TARGET_RAD_PER_SEC = 5.00f;

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

  motor_right.begin(KP_RIGHT, KI_RIGHT, KD_RIGHT);
  motor_left.begin(KP_LEFT, KI_LEFT, KD_LEFT);

  // micro-ROS setup
  if (!telemetry.begin()) {
    while (1) {
      Serial.println("micro-ROS init failed");
      delay(1000);
    }
  }
  delay(1000);
  start_time = millis();
}

void loop() {
  static unsigned long last_control_time = 0;
  static unsigned long last_print_time = 0;
  static unsigned long last_odom_pub_time = 0;
  static float omega_r = 0.0f;
  static float omega_l = 0.0f;

  telemetry.spin();
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

      float target_r = telemetry.getTargetRightRadPerSec();
      float target_l = telemetry.getTargetLeftRadPerSec();

      motor_right.setTargetRadPerSec(target_r);
      motor_left.setTargetRadPerSec(target_l);

      motor_right.update(omega_r, dt, RIGHT_ENCODER_INVERT);
      motor_left.update(omega_l, dt, LEFT_ENCODER_INVERT);
      odom.update(omega_r, omega_l, dt);
    }
  } 
  
  if(now -last_odom_pub_time >= ODOM_PUBLISH_PERIOD){
    last_odom_pub_time = now;
    telemetry.publishOdom(odom.getX(), odom.getY(), odom.getTheta());
  }
  
  if (now - last_print_time >= MEASURE_PERIOD) {
    last_print_time = now;

    int cmd_r = motor_right.getLastCommand();
    int cmd_l = motor_left.getLastCommand();

    float control_r = motor_right.getLastControl();
    float control_l = motor_left.getLastControl();

    Serial.print(" | x: ");
    Serial.print(odom.getX(), 3);

    Serial.print(" y: ");
    Serial.print(odom.getY(), 3);

    Serial.print(" theta: ");
    Serial.print(odom.getTheta(), 3);

    Serial.print(" target R: ");
    Serial.print(telemetry.getTargetRightRadPerSec());

    Serial.print(" target L: ");
    Serial.print(telemetry.getTargetLeftRadPerSec());

    Serial.print(" | R omega: ");
    Serial.print(omega_r);
    Serial.print("  control: ");
    Serial.print(control_r);
    Serial.print("  cmd: ");
    Serial.print(cmd_r);

    Serial.print(" | L omega: ");
    Serial.print(omega_l);
    Serial.print("  control: ");
    Serial.print(control_l);
    Serial.print("  cmd: ");
    Serial.println(cmd_l);
  }
}
