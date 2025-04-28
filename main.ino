#include "globals.h"
#include "servomotors.h"
#include "controlcommunications.h"

void setup() {
  initDisplay();
  initServo();             // Servo init + calibration
  
  calibrare_reed();

  setupControlSystem();    // PCA9685, I2C, display, etc.
}

void loop() {
  updateControlSystem();   // Your former loop logic
}
