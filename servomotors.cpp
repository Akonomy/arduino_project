#include <Arduino.h>
#include <Servo.h>
#include <math.h>  // pentru fabs()
#include "globals.h"

/////////////////////////////////////////
// Definiții pentru servo10 (codul 2)
/////////////////////////////////////////

// Structură de configurare pentru servo10.
struct ServoData {
  uint8_t id;               // Identificatorul servo-ului
  uint8_t defaultAngle;     // Unghiul de pornire (ex: 3°)
  int currentSensorPin;     // Pinul analogic pentru citirea curentului (ex: A2)
  uint8_t controlPin;       // Pinul digital de control (ex: 10)
};

ServoData servo10;     // Configurare pentru servo10
Servo myServo10;       // Obiect Servo pentru servo10

// Parametrii de mișcare și calibrare pentru servo10.
const uint8_t angleStepIncrease = 3;   // Pas creștere unghi
const uint8_t angleStepDecrease = 5;   // Pas descreștere unghi
const int startAngle = 3;              // Pozitia default (3°)
const int limitPhase1 = 35;            // Sfârșitul intervalului faza 1
const int limitPhase2 = 160;           // Sfârșitul intervalului de măsurare a curentului maxim
const int maxAngle10 = 177;            // Unghiul maxim pentru servo10
const float safeCurrentThreshold = 4.0;       // Prag sigur: 4A
const float emergencyCurrentThreshold = 6.0;    // Prag de urgență: 6A
const int over4AAllowed = 3;           // Număr maxim de măsurători consecutive peste 4A
const int over6AAllowed = 2;           // Număr maxim de măsurători consecutive peste 6A


/////////////////////////////////////////
// Definiții pentru servo9 (codul 1)
/////////////////////////////////////////

Servo myServo9;                     // Obiect Servo pentru servo9
const int pinServo9 = 9;            // Pinul de control pentru servo9
const int pinSensor9 = A3;          // Pinul analogic pentru senzorul asociat servo9


/////////////////////////////////////////
// Funcție de măsurare a curentului (servo10)
/////////////////////////////////////////
float measureCurrent(int pin) {
  float sum = 0;
  const int samples = 20;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  float avg = sum / samples;
  // Conversie folosind formula specificată
  return fabs((2.5 - (avg * 5.0 / 1024.0)) / 0.185);
}

/////////////////////////////////////////
// Funcția de calibrare pentru servo10
/////////////////////////////////////////
void runCalibration() {
  int consecutiveOver4A = 0;
  int consecutiveOver6A = 0;
  float maxMeasuredCurrent = 0.0;
  
  // Pasul 1: Mută servo10 la poziția default
  myServo10.write(servo10.defaultAngle);
  delay(500);
  
  // Faza 1: Creștere de la default la limitPhase1 cu pași de 3°
  for (int angle = servo10.defaultAngle; angle <= limitPhase1; angle += angleStepIncrease) {
    myServo10.write(angle);
    delay(10);
    float currentVal = measureCurrent(servo10.currentSensorPin);
    if (currentVal > safeCurrentThreshold)
      consecutiveOver4A++;
    else
      consecutiveOver4A = 0;
      
    if (currentVal > emergencyCurrentThreshold)
      consecutiveOver6A++;
    else
      consecutiveOver6A = 0;
    
    if (consecutiveOver6A > over6AAllowed || consecutiveOver4A > over4AAllowed)
      break;
  }
  
  // Faza 2: De la limitPhase1 la limitPhase2 – înregistrarea valorii maxime de curent
  for (int angle = limitPhase1; angle <= limitPhase2; angle += angleStepIncrease) {
    myServo10.write(angle);
    delay(10);
    float currentVal = measureCurrent(servo10.currentSensorPin);
    if (currentVal > maxMeasuredCurrent)
      maxMeasuredCurrent = currentVal;
    
    if (currentVal > emergencyCurrentThreshold)
      consecutiveOver6A++;
    else
      consecutiveOver6A = 0;
      
    if (consecutiveOver6A > over6AAllowed)
      break;
  }
  
  // Faza 3: De la limitPhase2 la unghiul maxim (maxAngle10) – doar măsurare
  for (int angle = limitPhase2; angle <= maxAngle10; angle += angleStepIncrease) {
    myServo10.write(angle);
    delay(10);
    float currentVal = measureCurrent(servo10.currentSensorPin);
    
    if (currentVal > emergencyCurrentThreshold)
      consecutiveOver6A++;
    else
      consecutiveOver6A = 0;
      
    if (consecutiveOver6A > over6AAllowed)
      break;
  }
  
  // Faza 4: Reducere de la maxAngle10 până aproape de default, cu pași de 5°
  for (int angle = maxAngle10; angle >= servo10.defaultAngle + 1; angle -= angleStepDecrease) {
    myServo10.write(angle);
    delay(10);
    float currentVal = measureCurrent(servo10.currentSensorPin);
    
    if (currentVal > emergencyCurrentThreshold)
      consecutiveOver6A++;
    else
      consecutiveOver6A = 0;
      
    if (consecutiveOver6A > over6AAllowed)
      break;
  }
  
  // Asigură-te că servo10 ajunge exact la poziția default
  myServo10.write(servo10.defaultAngle);
  delay(500);
  
  // Salvează valoarea maximă măsurată
  calibMaxCurrent = maxMeasuredCurrent+0.14;
  showVoltage(calibMaxCurrent);
  delay(1000);
  
  // Detach pentru a termina calibrarea
  myServo10.detach();
}

/////////////////////////////////////////
// Funcția de mișcare pentru servo10
/////////////////////////////////////////
int moveServo10(uint8_t state) {
  if (state == 0) {
    // Deplasare spre poziția default (3°), reducând în pași de 3°
    for (int angle = currentAngle10; angle > servo10.defaultAngle; angle -= 3) {
      myServo10.write(angle);
      delay(15);
      currentAngle10 = angle;
    }
    myServo10.write(servo10.defaultAngle);
    currentAngle10 = servo10.defaultAngle;
    return currentAngle10;
  }
  else if (state == 1) {
    // Mișcare spre un unghi mai mare (până la ~157°) cu verificare curent
    for (int angle = currentAngle10; angle <= 157; angle += 3) {
      myServo10.write(angle);
      delay(25);
      float currentVal = measureCurrent(servo10.currentSensorPin);
      showVoltage(currentVal); // adaugă după citire
      
      // Dacă, în intervalul 40°–144°, curentul depășește calibMaxCurrent,
      // retragere la poziția default (3°)
      if (angle >= 40 && angle <= 144 && currentVal > calibMaxCurrent) {
        for (int ret = angle; ret > servo10.defaultAngle; ret -= 3) {
          myServo10.write(ret);
          delay(20);
          currentAngle10 = ret;
        }
        myServo10.write(servo10.defaultAngle);
        currentAngle10 = servo10.defaultAngle;
        return currentAngle10;
      }
      
      // Dacă, în intervalul 145°–157°, curentul depășește calibMaxCurrent,
      // se oprește mișcarea și se returnează unghiul curent
      if (angle >= 145 && angle <= 157 && currentVal > calibMaxCurrent) {
        currentAngle10 = angle;
        return angle;
      }
      currentAngle10 = angle;
    }
    return currentAngle10;
  }
  return currentAngle10;
}

/////////////////////////////////////////
// Funcția de mișcare pentru servo9
/////////////////////////////////////////
int moveServo9(uint8_t state) {
  if (state == 0) {
    // Deplasare spre 177° (creștere)
    for (int angle = currentAngle9; angle <= 177; angle++) {
      myServo9.write(angle);
      delay(10);
    }
    currentAngle9 = 177;
    return currentAngle9;
  }
  else if (state == 1) {
    // Deplasare spre 3° (scădere) cu pași de 3° și verificare senzor (analogRead pe A3)
    for (int angle = currentAngle9; angle >= 3; angle -= 3) {
      myServo9.write(angle);
      delay(20);
      int sensorValue = analogRead(pinSensor9);
      if (sensorValue > 10) {  
        // Retragere imediată: de la unghiul curent către 170° în pași de 3°
        for (int retract = angle; retract <= 170; retract += 3) {
          myServo9.write(retract);
          delay(20);
        }
        currentAngle9 = 170;
        return currentAngle9;
      }
    }
    delay(100);
    int finalCheck = analogRead(pinSensor9);
    if (finalCheck > 10) {
      for (int retract = 3; retract <= 170; retract += 3) {
        myServo9.write(retract);
        delay(20);
      }
      currentAngle9 = 170;
      return currentAngle9;
    }
    currentAngle9 = 3;
    return currentAngle9;
  }
  return currentAngle9;
}

/////////////////////////////////////////
// Funcția generală seteazaServo(servo, state)
// - Dacă servo == 10, apelează funcția moveServo10()
// - Dacă servo == 9, apelează funcția moveServo9()
/////////////////////////////////////////
int seteazaServo(uint8_t servo, uint8_t state) {

  //Variabile trimise prin I2c
  lastServoResponse = 101;  // default magic value dacă nu s-a făcut nimic
  int result = -1;

  if (servo == 10) {
    myServo10.attach(servo10.controlPin);
    result = moveServo10(state);

    if ((state == 1 && result > 130) || (state == 0 && result < 10)) {
      return 1;  // condition met
    } else {
      return 0;  // condition failed
    }
  }
  else if (servo == 9) {
    myServo9.attach(pinServo9);
    result = moveServo9(state);

    if ((state == 1 && result < 10) || (state == 0 && result > 120)) {
      return 1;
    } else {
      return 0;
    }
  }

  return 104; // Unknown servo ID? Return 0 to signal failure because obviously.
}

/////////////////////////////////////////
// Funcția initServo()
// - Setează servo10 la 3° și servo9 la 177°
// - Apelează calibrarea pentru servo10 și salvează valoarea maximă
/////////////////////////////////////////
void initServo() {
  // Configurare pentru servo10
  servo10.id = 10;
  servo10.defaultAngle = startAngle;       // 3°
  servo10.currentSensorPin = A2;             // Senzorul de curent pentru servo10
  servo10.controlPin = 10;                   // Pinul de control pentru servo10
  currentAngle10 = servo10.defaultAngle;
  
  myServo10.attach(servo10.controlPin);
  myServo10.write(servo10.defaultAngle);
  
  // Configurare pentru servo9
  myServo9.attach(pinServo9);
  myServo9.write(177);
  currentAngle9 = 177;
  
  delay(500); // Pauză scurtă
  
  // Apelă calibrarea pentru servo10
  runCalibration();
}

/////////////////////////////////////////
// setup() și loop()
/////////////////////////////////////////

void calibrare_reed() {
  const int sweepStart = 3;
  const int sweepEnd = 160;
  const int sweepStep = 3;
  const int tolerance = 10; // ±10 grade în jurul unghiului găsit
  const int passes = 3;     // 3 măsurători de timp

  int firstDetectedAngle = -1;
  unsigned long measuredTimes[passes];
  uint8_t timeCount = 0;

  myServo10.attach(servo10.controlPin);

  // ======== Pas 1: Detectare unghi reed =========
  myServo10.write(sweepStart);
  delay(500);

  for (int angle = sweepStart; angle <= sweepEnd; angle += sweepStep) {
    myServo10.write(angle);
    float currentVal = measureCurrent(servo10.currentSensorPin);

    if (currentVal > 5.0) {
      myServo10.write(sweepStart);
      myServo10.detach();
      return; // Protecție overcurrent
    }

    delay(20);

    if (digitalRead(REED_SENSOR_PIN) == HIGH) {
      firstDetectedAngle = angle;
      showServoCode(firstDetectedAngle);
      delay(1000);
      break;
    }
  }

  if (firstDetectedAngle == -1) {
    // Dacă nu a fost detectat nimic
    myServo10.write(sweepStart);
    myServo10.detach();
    return;
  }

  // ======== Pas 2: Măsurare timp în fereastră ±10° =========

  while (timeCount < passes) {
    myServo10.write(sweepStart);
    delay(500);

    unsigned long startMillis = millis();
    bool detected = false;

    for (int angle = sweepStart; angle <= sweepEnd; angle += sweepStep) {
      myServo10.write(angle);
      float currentVal = measureCurrent(servo10.currentSensorPin);

      if (currentVal > 5.0) {
        myServo10.write(sweepStart);
        myServo10.detach();
        return;
      }

      delay(20);

      // Doar dacă suntem în fereastra de interes
      if (angle >= (firstDetectedAngle - tolerance) && angle <= (firstDetectedAngle + tolerance)) {
        if (digitalRead(REED_SENSOR_PIN) == HIGH) {
          measuredTimes[timeCount] = millis() - startMillis;
          showVoltage(measuredTimes[timeCount] / 1000.0);
          delay(1500);
          detected = true;
          break;
        }
      }
    }

    if (!detected) {
      // Dacă nu detectezi, măcar nu rămâne blocat
      measuredTimes[timeCount] = 9999;
    }

    timeCount++;
  }

  myServo10.write(sweepStart);
  delay(500);
  myServo10.detach();

  // ======== Pas 3: Selectăm timpul maxim şi aplicăm +1% =========
  unsigned long maxTime = 0;
  for (uint8_t i = 0; i < passes; i++) {
    if (measuredTimes[i] > maxTime && measuredTimes[i] != 9999) {
      maxTime = measuredTimes[i];
    }
  }

  if (maxTime > 0) {
    maxTime = (unsigned long)(maxTime * 1.01); // Adăugăm 1%
    showVoltage(maxTime / 1000.0);
    delay(2000);
  }

  reedDetectedAngleGlobal = firstDetectedAngle;
reedDetectionTimeMS = maxTime;

}



