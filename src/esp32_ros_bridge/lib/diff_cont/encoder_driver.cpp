#include "encoder_driver.h"
#include <Arduino.h>
// Biến volatile để xử lý trong ngắt
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(LEFT_ENC_PIN_B) == HIGH)
    leftEncoderCount++;
  else
    leftEncoderCount--;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_PIN_B) == HIGH)
    rightEncoderCount++;
  else
    rightEncoderCount--;
}

void initEncoder() {
  pinMode(LEFT_ENC_PIN_A, INPUT);
  pinMode(LEFT_ENC_PIN_B, INPUT);
  pinMode(RIGHT_ENC_PIN_A, INPUT);
  pinMode(RIGHT_ENC_PIN_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), rightEncoderISR, RISING);
}

long readEncoder(int i) {
  if (i == 0) return leftEncoderCount;
  else if (i == 1) return rightEncoderCount;
  return 0;
}

void resetEncoder(int i) {
  if (i == 0) leftEncoderCount = 0;
  else if (i == 1) rightEncoderCount = 0;
}

void resetEncoders() {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
}
