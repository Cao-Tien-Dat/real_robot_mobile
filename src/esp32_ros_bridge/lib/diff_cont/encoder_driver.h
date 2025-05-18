#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>

#define LEFT_ENC_PIN_A 16  
#define LEFT_ENC_PIN_B 17 

#define RIGHT_ENC_PIN_A 32  
#define RIGHT_ENC_PIN_B 33

void initEncoder();
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

#endif
