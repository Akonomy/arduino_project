#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <stdint.h>

// Servo Pins
#define SERVO10_PIN     10
#define SERVO10_SENSOR  A2
#define SERVO9_PIN      9
#define SERVO9_SENSOR   A3

// PCA + System Pins
#define OE_PIN           5
#define LED_PIN          7
#define PIN8             6
#define LOW_BATTERY_LED 13

// senzor reed
#define REED_SENSOR_PIN  4  

// Voltage Monitor
extern float batteryVoltage;
extern float prevBatteryVoltage;
extern int ultimaMedia;
extern bool safeMode;
extern bool measurementInProgress;

// Servo Angles
extern int currentAngle10;
extern int currentAngle9;
extern float calibMaxCurrent;

//Variabile trimise prin I2c
extern volatile uint8_t lastServoResponse;  // default magic value dacă nu s-a făcut nimic






extern bool reedActivated ;        // Flag dacă am detectat HIGH-ul
extern unsigned long startMillis ; // Start pentru cronometrat timpul
extern int reeddetectedAngle ;       // Unghiul unde am detectat


// Adaugă astea global, în globals.h sau înainte de setup():

extern int reedDetectedAngleGlobal; // unghiul unde a fost găsit reed-ul
extern unsigned long reedTimeLimitMS; // timpul (ms) până la reed (+1%)



void initDisplay(void);
void showVoltage(float voltage);
void showServoCode(uint8_t code);






#endif


