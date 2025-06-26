#include <Wire.h>
#include "PCA9685.h"
#include <TM1637Display.h>

#include "controlcommunications.h"
#include "globals.h"
#include "servomotors.h"



// ================== PCA9685 și I2C ==================
// Configurații hardware PCA9685/I2C

// Obiectul PCA9685
PCA9685 pwm = PCA9685(PCA9685_ADDRESS);

// Variabile volatile pentru datele primite prin I²C
volatile bool dataReceived = false;
volatile uint16_t receivedMask = 0;
volatile uint16_t receivedValues[16]; // Valori PWM pentru canalele active
volatile uint8_t receivedNumValues = 0; // Numărul de valori primite






// Variabile pentru stocarea ultimului pachet (pentru a evita actualizările inutile)
uint16_t lastMask = 0;
uint16_t lastValues[16];
uint8_t lastNumValues = 0;

// Timestamp pentru ultima primire de date
unsigned long lastReceiveTime = 0;
// Flag pentru a semnala că s-a emis comanda de stop
bool stopIssued = false;
unsigned long stopTime = 0;

// Variabile globale pentru controlul PIN8
bool pin8State = false;  // Starea curentă a pinului 8 (false = LOW, true = HIGH)

// Variabile pentru secvența timeout la PIN8
bool timeoutSequenceActive = false;  // Indică dacă secvența timeout este în curs
int pin8TimeoutState = 0;            // 0: inactiv, 1: off pentru 1 sec, 2: on pentru 2 sec, 3: terminat
unsigned long pin8TimeoutStart = 0;  // Timestamp pentru schimbările din secvența timeout

// ================== Funcții I²C ==================
void requestEvent() {
  Wire.write(lastServoResponse);  // trimite către STM32 valoarea ultimului răspuns
}



void receiveEvent(int numBytes) {
  if (numBytes < 2) return; // Trebuie să avem cel puțin 2 octeți pentru mască

  uint8_t highByte = Wire.read();
  uint8_t lowByte  = Wire.read();
  uint16_t mask = ((uint16_t)highByte << 8) | lowByte;

  // Calcularea numărului de pini activi conform măștii
  uint8_t activeCount = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (mask & (1 << i))
      activeCount++;
  }
  int remainingBytes = numBytes - 2;
  if (remainingBytes % 2 != 0) return;  // Număr impar de octeți – date invalide
  uint8_t numValuesReceived = remainingBytes / 2;

  // Validare pachet:
  bool allowed = false;
  if (numValuesReceived == 1) {
    allowed = true;
  } else if (activeCount <= 4 && (numValuesReceived == activeCount || numValuesReceived == 4)) {
    allowed = true;
  } else if (activeCount > 4 && numValuesReceived == activeCount) {
    allowed = true;
  }
  
  if (!allowed) {
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }

  bool singleValuePacket = (numValuesReceived == 1);
  uint16_t values[16];
  
  if (singleValuePacket) {
    uint8_t hb = Wire.read();
    uint8_t lb = Wire.read();
    uint16_t singleValue = ((uint16_t)hb << 8) | lb;
    for (uint8_t i = 0; i < activeCount; i++) {
      values[i] = singleValue;
    }
  } else {
    uint8_t valuesToUse = (numValuesReceived == 4 && activeCount < 4) ? activeCount : numValuesReceived;
    for (uint8_t i = 0; i < valuesToUse; i++) {
      uint8_t hb = Wire.read();
      uint8_t lb = Wire.read();
      values[i] = ((uint16_t)hb << 8) | lb;
    }
  }

  noInterrupts();
  receivedMask = mask;
  receivedNumValues = singleValuePacket ? 1 : ((numValuesReceived == 4 && activeCount < 4) ? activeCount : numValuesReceived);
  for (uint8_t i = 0; i < (singleValuePacket ? activeCount : receivedNumValues); i++) {
    receivedValues[i] = values[i];
  }
  dataReceived = true;
  lastReceiveTime = millis();
  stopIssued = false;  // Resetăm comanda de stop dacă primim date
  interrupts();
}







// ================== TM1637 wir Măsurători ADC ==================
// Pinii pentru TM1637 sunt mutați pe 11 (CLK) și 12 (DIO)
TM1637Display display(11, 12);

// Parametri pentru măsurători ADC
const int numarMasuratori = 5;                  // Numărul de citiri efectuate
const unsigned long intrevalMasurare = 100;     // Interval între citiri (ms)
// Intervalul de măsurători este acum 16 minute (1000.000 ms)
const unsigned long intervalTotal = 1000000;      

// Variabile globale pentru măsurători ADC
unsigned long ultimulSetMasuratori = 0;


// FUNTIE MANUAL OVERRIDE


/**
 * Funcție blocking pentru setarea manuală a tensiunii cu debounce rapid (20 ms)
 * și ieșire după 2 s de stare stabilă.
 */
float manualVoltageSelection() {
  float Avoltage = 7.5;
  float AminVoltage = 6.7;
  float AmaxVoltage = 8.2;

  unsigned long AlastTransitionTime = millis();
  int AlastButtonState = digitalRead(MANUAL_OVERWRITE);
  bool AvalueUpdated = true;

  showVoltage(Avoltage);  // Arată tensiunea inițială

  while (true) {
    int currentButtonState = digitalRead(MANUAL_OVERWRITE);
    showVoltage(10.33);
    delay(10);
    
    // Detect transition LOW → HIGH
    if (currentButtonState == HIGH ) {
      Avoltage += 0.1;
      if (Avoltage > AmaxVoltage) {
        Avoltage = AminVoltage;
      }
      showVoltage(Avoltage);
      delay(500);
      showVoltage(55.99);
      delay(100);
      AlastTransitionTime = millis();
    }

    AlastButtonState = currentButtonState;

    // Dacă au trecut 2.5 secunde fără apăsări → finalizează
    if (millis() - AlastTransitionTime >= 2500) {
      return Avoltage;
    }

    delay(5);
  }
}











// ================== Funcții de măsurare ADC și siguranță ==================








// Efectuează 5 măsurători cu 100ms interval și actualizează 'ultimaMedia'
void efectueazaMasuratori() {
  int masuratori[numarMasuratori];
  unsigned long startMasurare = millis();
  
  for (int i = 0; i < numarMasuratori; i++) {
    while (millis() - startMasurare < i * intrevalMasurare) {
      // Așteptare activă
    }
    // Citim de pe A0
    masuratori[i] = analogRead(A0);
  }
  
  long suma = 0;
  for (int i = 0; i < numarMasuratori; i++) {
    suma += masuratori[i];
  }
  int media = suma / numarMasuratori;
  ultimaMedia = media;
  ultimulSetMasuratori = millis();
}

// Funcție care efectuează o măsurătoare unică (folosind 5 citiri) și returnează tensiunea în volți
float performSingleADCVoltageMeasurement() {
  efectueazaMasuratori();
  // Formula: V = ADC_media * 0.0180
  return ultimaMedia * 0.0180;
}

// Funcția de siguranță: efectuează 3 măsurători cu 10 sec între ele,
// repetând ciclul până când variația dintre ele este sub 0.5V (ideal ±0.2V)
float safetyMeasurementCycle() {
  float m1, m2, m3;
  while (true) {
    m1 = performSingleADCVoltageMeasurement();
    delay(10000);
    m2 = performSingleADCVoltageMeasurement();
    delay(10000);
    m3 = performSingleADCVoltageMeasurement();
    float maxVal = max(m1, max(m2, m3));
    float minVal = min(m1, min(m2, m3));
    if ((maxVal - minVal) <= 0.5) {
      return (m1 + m2 + m3) / 3.0;
    }
    // Dacă variația este prea mare, ciclul se repetă
  }
}

// Ciclu de măsurători care suspendă controlul PCA, așteaptă stabilizarea tensiunii,
// efectuează măsurătoarea și, dacă este necesar, activează modul de siguranță.
void measurementCycle() {
  measurementInProgress = true;
  
  // Suspendă controlul PCA: dezactivează ieșirile
  digitalWrite(OE_PIN, HIGH);
  // Așteaptă 1000ms pentru stabilizarea tensiunii bateriei
  delay(1000);
  
  // Efectuează o măsurătoare
  efectueazaMasuratori();
  float measuredVoltage = ultimaMedia * 0.0180;
  if (measuredVoltage>9){
    measuredVoltage=7.12;
  }
  
  // Dacă tensiunea variază prea mult de la valoarea anterioară (ex. scade cu mai mult de 0.5V)
  // sau este sub limita critică (6.6V), se intră în modul de siguranță.
  if ((prevBatteryVoltage - measuredVoltage) > 0.5 || measuredVoltage < 6.6) {
    safeMode = true;
  }
  
  // Dacă suntem în modul de siguranță, efectuăm ciclul de măsurători de siguranță:
  if (safeMode) {
    // Aprinde LED-ul de avertizare (pinul 13)
    digitalWrite(LOW_BATTERY_LED, HIGH);
    // Se repetă setul de 3 măsurători până când variația este acceptabilă
    batteryVoltage = safetyMeasurementCycle();
  } else {
    batteryVoltage = measuredVoltage;
  }
  
  // Actualizează valoarea de referință pentru viitoarele comparări
  prevBatteryVoltage = batteryVoltage;
  
  // Se revine la controlul PCA: activează ieșirile
  digitalWrite(OE_PIN, LOW);
  
  // Dacă tensiunea s-a normalizat (de exemplu, peste 6.6V), se iese din modul de siguranță
  if (batteryVoltage >= 6.6) {
    safeMode = false;
    digitalWrite(LOW_BATTERY_LED, LOW);
  }

    if (batteryVoltage >= 8.5) {
    batteryVoltage=7.19;
    safeMode = false;
    digitalWrite(LOW_BATTERY_LED, LOW);
  }
  
  measurementInProgress = false;
}

// ================== Setup ==================
void setupControlSystem() {
  // Inițializare I2C și eveniment de recepție
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent); 
  Wire.setClock(400000);  // 400 kHz
  
  // Configurări pini PCA9685 și indicatori
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(PIN8, OUTPUT);
  digitalWrite(PIN8, LOW);
  pinMode(LOW_BATTERY_LED, OUTPUT);
  digitalWrite(LOW_BATTERY_LED, LOW);
  
  // Inițializează PCA9685
  pwm.begin();
  pwm.setPWMFreq(100);
  
  // Inițializare variabile pentru pachetele I²C
  lastMask = 0;
  lastNumValues = 0;
  lastReceiveTime = millis();
  
  // Efectuează prima măsurătoare ADC pentru a seta tensiunea de referință
  efectueazaMasuratori();
  prevBatteryVoltage = batteryVoltage = ultimaMedia * 0.0180;
}

// ================== Loop ==================
void updateControlSystem() {

  unsigned long now = millis();
  static unsigned long manualTimer = 0;
  static int manualState = LOW;


static unsigned long MANUAL_lastCheckTime = 0;
const unsigned long MANUAL_checkInterval = 75; // ms – pentru protecția nervilor procesorului

int readPin = digitalRead(MANUAL_OVERWRITE);

if (now - MANUAL_lastCheckTime >= MANUAL_checkInterval) {
  MANUAL_lastCheckTime = now;

  if (readPin == HIGH && manualState == LOW) {
    // Intrare instantă în modul manual fără delay emoțional
    showVoltage(1.09);
    delay(500);  // Înainte să intrăm în zona de selecție

    float newV = manualVoltageSelection();
    batteryVoltage = newV;
    prevBatteryVoltage = newV;

    manualState = HIGH;
    manualTimer = now;
  } 
  else if (readPin != manualState) {
    manualState = readPin;
  }
}




  unsigned long currentTime = millis();




  
  // ---------- Procesare evenimente I²C și actualizare PCA9685 ----------
  // Dacă nu suntem în modul de siguranță și nu se efectuează măsurători,
  // se procesează evenimentele I²C.
  if (!measurementInProgress && !safeMode) {
    if (dataReceived) {
      timeoutSequenceActive = false;
      pin8TimeoutState = 0;
      
      // Toggle pe PIN8
      pin8State = !pin8State;
      digitalWrite(PIN8, pin8State ? HIGH : LOW);
      
      // Copiem datele primite în variabile locale
      noInterrupts();
      uint16_t mask = receivedMask;
      uint8_t numVal = receivedNumValues;
      uint16_t localValues[16];
      for (uint8_t i = 0; i < numVal; i++) {
        localValues[i] = receivedValues[i];
      }
      dataReceived = false;
      interrupts();
      
      // Comparăm cu ultimul pachet pentru a vedea dacă s-au schimbat valorile
      bool different = (mask != lastMask) || (numVal != lastNumValues);
      if (!different) {
        for (uint8_t i = 0; i < numVal; i++) {
          if (localValues[i] != lastValues[i]) {
            different = true;
            break;
          }
        }
      }
      
      // Controlul LED-ului (PIN7)
      if (!different) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
      
      // Aplicăm factorul de corecție asupra valorilor PWM primite.
      // Factorul de corecție este: 8.0 / batteryVoltage (testat la 8V).
      float correctionFactor = 8.0 / batteryVoltage;
      for (uint8_t i = 0; i < numVal; i++) {
        uint16_t corrected = (uint16_t)(localValues[i] * correctionFactor);
        if (corrected > 4096) corrected = 4096;
        localValues[i] = corrected;
      }



// Check if the command is for servo 9 or 10
bool isServoCommand = (mask == 0x0200 || mask == 0x0400);
uint8_t servoId = (mask == 0x0200) ? 9 : (mask == 0x0400) ? 10 : 255;

if (isServoCommand && servoId != 255) {
    // Extragem comanda exactă
    uint8_t state = localValues[0]; // poate fi 0, 1, 2 sau 3

    // DEBUG: Afișăm pe TM1637 starea primită pentru servo
    showServoCode(state);
   

    //

    switch (state) {
      case 0:
      case 1:
        // Execută mișcarea normală
        lastServoResponse = seteazaServo(servoId, state);
        break;

      case 2:
        // Recalibrare doar pentru servo10
       
          initServo();  // Setează la poziția inițială + calibrări
          lastServoResponse = 1;
      
        break;

      case 3:
        // Detach only if it deserves it. Like my respect.
        if (servoId == 10) {
          myServo10.detach();
        } else if (servoId == 9) {
          myServo9.detach();
        }

        lastServoResponse = 1; // detachment, considerat ok

        break;

      case 4:
        // Execută mișcarea normală
        lastServoResponse = seteazaServo(servoId, state);
        break;

      default:
        // Valoare necunoscută, ignorăm
        lastServoResponse = 105; // 104 not found
        break;
    }

    // LED feedback
    if ((mask != lastMask) || (localValues[0] != lastValues[0])) {
        digitalWrite(LED_PIN, LOW);
    } else {
        digitalWrite(LED_PIN, HIGH);
    }

    // Update last received state
    lastMask = mask;
    lastNumValues = 1;
    lastValues[0] = localValues[0];
    lastReceiveTime = currentTime;

    // Sar peste PCA9685 logică
    return;
} // end f conditie if masca servo 9 sau 10


else if (mask == 0x0800) {  // 1 << 11
  adjust_box();  // ajustare mecanică
  lastMask = mask;
  lastNumValues = 1;
  lastValues[0] = localValues[0];
  lastReceiveTime = currentTime;
  return;
}





      
      // Dacă pachetul a diferit, actualizează PCA9685
      if (different) {
        if (numVal == 1) {
          pwm.setMultiplePinsSame(mask, localValues[0]);
        } else {
          pwm.setMultiplePins(mask, localValues);
        }
        lastMask = mask;
        lastNumValues = numVal;
        for (uint8_t i = 0; i < numVal; i++) {
          lastValues[i] = localValues[i];
        }
      }
      lastReceiveTime = currentTime;
    }
    
    // Verificare timeout refresh (300 ms)
    if ((currentTime - lastReceiveTime) > FALLBACK_TIMEOUT) {
      if (!stopIssued) {
        digitalWrite(OE_PIN, HIGH);
        pwm.setMultiplePinsSame(0xFFFF, 0);
        stopTime = currentTime;
        stopIssued = true;
      } else {
        if ((currentTime - stopTime) >= RELEASE_DELAY) {
          digitalWrite(OE_PIN, LOW);
          lastReceiveTime = currentTime;
        }
      }
      if (!timeoutSequenceActive) {
        timeoutSequenceActive = true;
        pin8TimeoutState = 1;         // OFF pentru 1 sec
        pin8TimeoutStart = currentTime;
        digitalWrite(PIN8, LOW);
        pin8State = false;
      }
    }
    
    // Secvență timeout pentru PIN8
    if (timeoutSequenceActive) {
      if (pin8TimeoutState == 1) { // OFF pentru 1 sec
        if (currentTime - pin8TimeoutStart >= 1000) {
          pin8TimeoutState = 2;
          pin8TimeoutStart = currentTime;
          digitalWrite(PIN8, HIGH);
          pin8State = true;
        }
      } else if (pin8TimeoutState == 2) { // ON pentru 2 sec
        if (currentTime - pin8TimeoutStart >= 2000) {
          pin8TimeoutState = 3;
          digitalWrite(PIN8, LOW);
          pin8State = false;
        }
      } else if (pin8TimeoutState == 3) {
        timeoutSequenceActive = false;
        pin8TimeoutState = 0;
      }
    }
  } // sfârșit procesare I²C, dacă nu e în modul de siguranță
  
  // ---------- Ciclu de măsurători ADC și afișare TM1637 ----------
  // Dacă a trecut intervalul de 5 minute, se intră în ciclul de măsurători,
  // care suspendă controlul PCA pentru a stabiliza tensiunea bateriei.
  if (currentTime - ultimulSetMasuratori >= intervalTotal && !measurementInProgress) {
    measurementCycle();
  }
  
  // Calculăm tensiunea pe baza formulei calibrate:
  // V_in = ADC_media * 0.0180  (V per unitate ADC)
  // Pentru afișare în format X.XX se înmulțește cu 100 (deci, V*100 = ADC_media * 1.8)
  showVoltage(batteryVoltage);

  

}



void adjust_box() {
  const uint16_t MASK_INAINTE = 0x0055; // înainte
  const uint16_t MASK_STANGA_FATA = 0x0009; // roți față spre dreapta
  const uint16_t MASK_DREAPTA_FATA = 0x0006; // roți față spre stânga
  const uint16_t MASK_STOP = 0x0000; // stop

  const uint16_t viteza_tick = 2500; // impuls scurt

  // Bop înainte
  pwm.setMultiplePinsSame(MASK_INAINTE, viteza_tick);
  delay(60); // 50 ms
  pwm.setMultiplePinsSame(MASK_STOP, 0);
  delay(50); // Pauză mică să nu fie prea *spicy*

  // Tap față dreapta
  pwm.setMultiplePinsSame(MASK_STANGA_FATA, viteza_tick);
  delay(80);
  pwm.setMultiplePinsSame(MASK_STOP, 0);
  delay(50);

  // Tap față stânga
  pwm.setMultiplePinsSame(MASK_DREAPTA_FATA, viteza_tick);
  delay(80);
  pwm.setMultiplePinsSame(MASK_STOP, 0);
  delay(50);

    pwm.setMultiplePinsSame(MASK_INAINTE, viteza_tick);
  delay(60); // 50 ms
  pwm.setMultiplePinsSame(MASK_STOP, 0);
  delay(50); // Pauză mică să nu fie prea *spicy*
}




