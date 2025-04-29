#ifndef CONTROL_COMMUNICATIONS_H
#define CONTROL_COMMUNICATIONS_H

#include <Arduino.h>


#define PCA9685_ADDRESS 0x40   // Adresa modulului PCA9685
#define SLAVE_ADDRESS   0x08    // Adresa Arduino ca I²C slave
#define OE_PIN          5       // Pinul de Output Enable (OE) – HIGH dezactivează ieșirile
#define FALLBACK_TIMEOUT 300    // Timeout de refresh (ms)
#define RELEASE_DELAY   50      // Timp după care se dezactivează OE (ms)

// Definirea pinilor pentru indicator și toggle timeout
#define LED_PIN 7             // LED-ul care indică pachet identic (I²C)
#define PIN8 8                // Pinul care se comută la primirea unui pachet și în timeout
#define LOW_BATTERY_LED 13    // LED pentru avertizare baterie scăzută


void setupControlSystem();  // to replace original setup()
void updateControlSystem(); // to replace original loop()

void adjust_box();

#endif
