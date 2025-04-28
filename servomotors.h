#ifndef SERVOMOTORS_H
#define SERVOMOTORS_H

#include <Arduino.h>
#include <Servo.h>

extern Servo myServo9;
extern Servo myServo10;



void initServo();
int seteazaServo(uint8_t servo, uint8_t state);
float measureCurrent(int pin);
void runCalibration();

void calibrare_reed();

#endif
