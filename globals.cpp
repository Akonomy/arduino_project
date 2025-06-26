
#include <TM1637Display.h>
#include "globals.h"

// Pinii pentru display
#define CLK_PIN 11
#define DIO_PIN 12


float batteryVoltage = 8.0;
float prevBatteryVoltage = 8.0;
int ultimaMedia = 0;
bool safeMode = false;
bool measurementInProgress = false;

int currentAngle10 = 3;
int currentAngle9 = 177;
float calibMaxCurrent = 2.73;




//Variabile trimise prin I2c
volatile uint8_t lastServoResponse = 56;  // default magic value dacă nu s-a făcut nimic

// pt senzor reed
bool reedActivated = false;        // Flag dacă am detectat HIGH-ul
unsigned long startMillis = 0; // Start pentru cronometrat timpul
int reeddetectedAngle = -1;       // Unghiul unde am detectat

// Inițializare la început de fișier
int reedDetectedAngleGlobal = 87;
unsigned long reedTimeLimitMS = 1300;



// Inițializare display
static TM1637Display display(CLK_PIN, DIO_PIN);

void initDisplay(void) {
    display.setBrightness(0x0f);
}

void showVoltage(float voltage) {
    // Ex: 7.89V -> 789
    int toDisplay = (int)(voltage * 100);
    display.showNumberDecEx(toDisplay, 0b01000000, false, 4, 0);
}

void showServoCode(uint8_t code) {
    display.showNumberDecEx(code, 0b00000000, true); // fără puncte
}

