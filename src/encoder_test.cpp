#include <Arduino.h>

#include "config.hpp"

namespace {

struct EncoderProbe {
  const uint8_t pin_a;
  const uint8_t pin_b;
  volatile uint8_t state;
  volatile int32_t count;
  volatile uint32_t transitions;
  volatile uint32_t invalid_transitions;
  volatile uint32_t state_visits[4];
};

struct ProbeSnapshot {
  uint8_t state;
  int32_t count;
  uint32_t transitions;
  uint32_t invalid_transitions;
  uint32_t state_visits[4];
};

EncoderProbe right_probe = {
    RIGHT_ENC_A, RIGHT_ENC_B, 0, 0, 0, 0, {0, 0, 0, 0}};
EncoderProbe left_probe = {
    LEFT_ENC_A, LEFT_ENC_B, 0, 0, 0, 0, {0, 0, 0, 0}};

constexpr uint32_t PRINT_PERIOD_MS = 100;
constexpr int TEST_PWM = PWM_MAX / 5;  // 20% duty
constexpr uint32_t MOTOR_RUN_MS = 5000;
uint32_t motor_stop_ms = 0;

void stopMotors() {
  analogWrite(RIGHT_PIN_1, 0);
  analogWrite(RIGHT_PIN_2, 0);
  analogWrite(LEFT_PIN_1, 0);
  analogWrite(LEFT_PIN_2, 0);
}

void driveMotor(uint8_t pin_forward, uint8_t pin_reverse, bool enabled) {
  analogWrite(pin_forward, enabled ? TEST_PWM : 0);
  analogWrite(pin_reverse, 0);
}

void printMotorHelp() {
  Serial.println("Commands: r=right, l=left, b=both, s=stop");
}

void handleMotorCommand() {
  if (motor_stop_ms != 0 && static_cast<int32_t>(millis() - motor_stop_ms) >= 0) {
    stopMotors();
    motor_stop_ms = 0;
    Serial.println("MOTOR: automatic stop after 5 seconds");
  }

  if (Serial.available() == 0) {
    return;
  }

  const char command = Serial.read();
  if (command == '\r' || command == '\n') {
    return;
  }

  stopMotors();
  motor_stop_ms = 0;
  switch (command) {
    case 'r':
    case 'R':
      driveMotor(RIGHT_PIN_1, RIGHT_PIN_2, true);
      motor_stop_ms = millis() + MOTOR_RUN_MS;
      Serial.println("MOTOR: right at 20%");
      break;
    case 'l':
    case 'L':
      driveMotor(LEFT_PIN_1, LEFT_PIN_2, true);
      motor_stop_ms = millis() + MOTOR_RUN_MS;
      Serial.println("MOTOR: left at 20%");
      break;
    case 'b':
    case 'B':
      driveMotor(RIGHT_PIN_1, RIGHT_PIN_2, true);
      driveMotor(LEFT_PIN_1, LEFT_PIN_2, true);
      motor_stop_ms = millis() + MOTOR_RUN_MS;
      Serial.println("MOTOR: both at 20%");
      break;
    case 's':
    case 'S':
      Serial.println("MOTOR: stopped");
      break;
    default:
      Serial.println("Unknown command; motors stopped");
      printMotorHelp();
      break;
  }
}

uint8_t readState(const EncoderProbe& probe) {
  const uint8_t a = digitalRead(probe.pin_a) == HIGH ? 1 : 0;
  const uint8_t b = digitalRead(probe.pin_b) == HIGH ? 1 : 0;
  return (a << 1) | b;
}

int8_t transitionDelta(uint8_t previous, uint8_t current) {
  // Positive sequence: 00 -> 01 -> 11 -> 10 -> 00.
  static constexpr int8_t table[16] = {
       0,  1, -1,  0,
      -1,  0,  0,  1,
       1,  0,  0, -1,
       0, -1,  1,  0,
  };
  return table[(previous << 2) | current];
}

void updateProbe(EncoderProbe& probe) {
  const uint8_t current = readState(probe);
  const uint8_t previous = probe.state;
  if (current == previous) {
    return;
  }

  probe.state = current;
  probe.transitions++;
  probe.state_visits[current]++;

  const int8_t delta = transitionDelta(previous, current);
  if (delta == 0) {
    probe.invalid_transitions++;
  } else {
    probe.count += delta;
  }
}

void isrRightA() { updateProbe(right_probe); }
void isrRightB() { updateProbe(right_probe); }
void isrLeftA() { updateProbe(left_probe); }
void isrLeftB() { updateProbe(left_probe); }

ProbeSnapshot snapshot(const EncoderProbe& probe) {
  noInterrupts();
  const ProbeSnapshot result = {
      probe.state,
      probe.count,
      probe.transitions,
      probe.invalid_transitions,
      {probe.state_visits[0], probe.state_visits[1],
       probe.state_visits[2], probe.state_visits[3]}};
  interrupts();
  return result;
}

void printProbe(const char* label, const ProbeSnapshot& probe) {
  Serial.print(label);
  Serial.print(" A=");
  Serial.print((probe.state >> 1) & 1);
  Serial.print(" B=");
  Serial.print(probe.state & 1);
  Serial.print(" count=");
  Serial.print(probe.count);
  Serial.print(" edges=");
  Serial.print(probe.transitions);
  Serial.print(" invalid=");
  Serial.print(probe.invalid_transitions);
  Serial.print(" states[00,01,10,11]=");
  Serial.print(probe.state_visits[0]);
  Serial.print(',');
  Serial.print(probe.state_visits[1]);
  Serial.print(',');
  Serial.print(probe.state_visits[2]);
  Serial.print(',');
  Serial.print(probe.state_visits[3]);
}

void beginProbe(EncoderProbe& probe) {
  pinMode(probe.pin_a, INPUT_PULLUP);
  pinMode(probe.pin_b, INPUT_PULLUP);
  probe.state = readState(probe);
  probe.state_visits[probe.state] = 1;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(RIGHT_PIN_1, OUTPUT);
  pinMode(RIGHT_PIN_2, OUTPUT);
  pinMode(LEFT_PIN_1, OUTPUT);
  pinMode(LEFT_PIN_2, OUTPUT);
  analogWriteResolution(PWM_BIT);
  analogWriteFrequency(RIGHT_PIN_1, PWM_FREQ_HZ);
  analogWriteFrequency(RIGHT_PIN_2, PWM_FREQ_HZ);
  analogWriteFrequency(LEFT_PIN_1, PWM_FREQ_HZ);
  analogWriteFrequency(LEFT_PIN_2, PWM_FREQ_HZ);
  stopMotors();

  beginProbe(right_probe);
  beginProbe(left_probe);

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isrRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), isrRightB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), isrLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), isrLeftB, CHANGE);

  Serial.println("Encoder and motor test (motors start stopped)");
  Serial.println("WARNING: lift the robot before starting a motor.");
  printMotorHelp();
  Serial.print("Pins: R(A,B)=");
  Serial.print(RIGHT_ENC_A);
  Serial.print(',');
  Serial.print(RIGHT_ENC_B);
  Serial.print(" L(A,B)=");
  Serial.print(LEFT_ENC_A);
  Serial.print(',');
  Serial.println(LEFT_ENC_B);
}

void loop() {
  static uint32_t last_print_ms = 0;
  handleMotorCommand();
  const uint32_t now = millis();
  if (now - last_print_ms < PRINT_PERIOD_MS) {
    return;
  }
  last_print_ms = now;

  const ProbeSnapshot right = snapshot(right_probe);
  const ProbeSnapshot left = snapshot(left_probe);

  Serial.print("t=");
  Serial.print(now);
  Serial.print(" | ");
  printProbe("R", right);
  Serial.print(" | ");
  printProbe("L", left);
  Serial.println();
}
