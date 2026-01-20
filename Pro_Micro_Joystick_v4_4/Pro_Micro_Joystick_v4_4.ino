// Custom Joystick Firmware
// VERSION 4.4: Adjustable Safety Angle & Menu Polish
// - Added Menu Option 10: "Safety Cutoff Angle".
// - Responsiveness (Kalman Level) now shows text descriptions in menu.
// - Safety Logic now uses the user-defined angle instead of hardcoded 80.

#include "Joystick.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>

//================================================================
// CONFIGURATION STRUCTURE
//================================================================
const String FIRMWARE_VERSION = "4.4"; 

struct JoystickConfig {
  int version;
  int leanAngle;
  bool isRightHanded;
  int kalmanLevel;
  int pickupThreshold;
  int rudderMode; 
  
  bool invertRoll;    
  bool invertPitch;   
  bool invertThrottle;
  
  bool smartSafety; 
  int safetyAngle; // NEW: Customizable Cutoff Angle
  
  // Pin Mappings
  uint8_t pinCoolieUp;
  uint8_t pinCoolieDown;
  uint8_t pinCoolieLeft;
  uint8_t pinCoolieRight;
  uint8_t pinCooliePress;
  uint8_t pinBtn1;
  uint8_t pinBtn4;
  uint8_t pinModLeft;  
  uint8_t pinModRight; 
  uint8_t pinSet;
  uint8_t pinRst;
};

JoystickConfig config;
const int CONFIG_VERSION = 11; // Version bump

//================================================================
// CONSTANTS & GLOBALS
//================================================================
const int THRUST_PIN      = A0; 
const int ACTIVATION_PIN  = 14; 
const uint8_t PIN_DISABLED = 255; 

const uint8_t candidatePins[] = {4, 5, 6, 7, 8, 9, 10, 15, 16, A3, 20}; 
const int numCandidates = 11;

Adafruit_MPU6050 mpu;
bool isImuActive = false;
int lastActivationButtonState = HIGH;

// State Variables
int thrustValue = 0;   
int xAxisValue = 0;    
int rudderValue = 128; 
int storedRudderValue = 128; 
int storedXAxisValue = 0;

bool potiControlEngaged = true;
bool lastModifierState = false;

// Safety Logic Variables
bool safetyCutoffActive = false; 
bool wasSafetyActive = false;
int safetyHoldPitch = 0;
int safetyHoldRudder = 128;

unsigned long setButtonTimer = 0;
bool setButtonActive = false;
bool throttleSafetyLock = false;

unsigned long firstClickTime = 0;
const unsigned int doubleClickWindow = 400;
bool waitingForSecondClick = false;

// Rolling Average Buffers
const int BUFFER_SIZE = 30; 
int pitchBuffer[BUFFER_SIZE];
int rudderBuffer[BUFFER_SIZE];
int bufferIndex = 0;
unsigned long lastBufferUpdate = 0;

// Kalman Globals
float kalman_angle_pitch = 0, kalman_unbiased_rate_pitch = 0, kalman_bias_pitch = 0;
float P_pitch[2][2] = { { 1, 0 }, { 0, 1 } };
float kalman_angle_roll = 0, kalman_unbiased_rate_roll = 0, kalman_bias_roll = 0;
float P_roll[2][2] = { { 1, 0 }, { 0, 1 } };
float Q_angle, Q_bias = 0.003, R_measure;
unsigned long timer;

// Forward Declarations
void loadConfiguration();
void saveConfiguration();
void applyKalmanSettings(int level);
void runSetupMenu();
void runInvertMenu(); 
void runDebugMenu();
void learnButtons();
float updateKalman(float newAngle, float newRate, float dt, float* kalman_angle, float* kalman_bias, float P[2][2]);
int calculateAverage(int* buffer, int size);
void initBuffers();

int readSmart(uint8_t pin) {
  if (pin == PIN_DISABLED) return HIGH; 
  return digitalRead(pin);
}

void pinModeSmart(uint8_t pin, uint8_t mode) {
  if (pin != PIN_DISABLED) pinMode(pin, mode);
}

//================================================================
// SETUP
//================================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(ACTIVATION_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  
  loadConfiguration();
  initBuffers();
  
  pinModeSmart(config.pinCoolieUp, INPUT_PULLUP);
  pinModeSmart(config.pinCoolieDown, INPUT_PULLUP);
  pinModeSmart(config.pinCoolieLeft, INPUT_PULLUP);
  pinModeSmart(config.pinCoolieRight, INPUT_PULLUP);
  pinModeSmart(config.pinCooliePress, INPUT_PULLUP);
  pinModeSmart(config.pinBtn1, INPUT_PULLUP);
  pinModeSmart(config.pinBtn4, INPUT_PULLUP);
  pinModeSmart(config.pinModLeft, INPUT_PULLUP);
  pinModeSmart(config.pinModRight, INPUT_PULLUP);
  pinModeSmart(config.pinSet, INPUT_PULLUP);
  pinModeSmart(config.pinRst, INPUT_PULLUP);
  
  pinMode(THRUST_PIN, INPUT);

  if (digitalRead(ACTIVATION_PIN) == LOW) {
    runSetupMenu();
  }

  Wire.begin();
  if (!mpu.begin()) {
    while (1) { 
        digitalWrite(LED_BUILTIN, HIGH); delay(100); 
        digitalWrite(LED_BUILTIN, LOW); delay(100); 
    }
  }
  timer = micros();
  Joystick.begin(false);
}

//================================================================
// MAIN LOOP
//================================================================
void loop() {
  unsigned long currentMillis = millis();

  // --- Hand-Swap Logic ---
  if (readSmart(config.pinSet) == LOW) {
    if (!setButtonActive) {
      setButtonActive = true;
      setButtonTimer = currentMillis;
    } else {
      if (currentMillis - setButtonTimer > 3000) {
        config.isRightHanded = !config.isRightHanded;
        saveConfiguration();
        throttleSafetyLock = true; 
        for(int i=0; i<5; i++) { digitalWrite(LED_BUILTIN, HIGH); delay(50); digitalWrite(LED_BUILTIN, LOW); delay(50); }
        while(readSmart(config.pinSet) == LOW) { delay(10); }
        setButtonActive = false;
      }
    }
  } else {
    setButtonActive = false;
  }

  // --- Activation Logic ---
  int currentActivationButtonState = digitalRead(ACTIVATION_PIN);
  if (lastActivationButtonState == HIGH && currentActivationButtonState == LOW) {
    if (waitingForSecondClick && (currentMillis - firstClickTime < doubleClickWindow)) {
      isImuActive = !isImuActive;
      waitingForSecondClick = false;
    } else {
      waitingForSecondClick = true;
      firstClickTime = currentMillis;
    }
  }
  lastActivationButtonState = currentActivationButtonState;

  if (waitingForSecondClick && (currentMillis - firstClickTime >= doubleClickWindow)) {
    waitingForSecondClick = false;
  }

  // --- IMU Reading ---
  int rawXAxis = 0; int rawYAxis = 0; int rawRollToRudder = 128; 
  float safety_roll = 0; float safety_pitch = 0;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (isImuActive) {
    digitalWrite(LED_BUILTIN, HIGH); 
    float dt = (float)(micros() - timer) / 1000000.0;
    timer = micros();
    
    float accel_pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
    float accel_roll = atan2(a.acceleration.y, -a.acceleration.z) * 180 / M_PI;
    safety_roll = accel_roll; safety_pitch = accel_pitch;

    float gyro_rate_pitch = g.gyro.y * 180 / M_PI;
    float gyro_rate_roll = g.gyro.x * 180 / M_PI;
    
    float filtered_pitch = updateKalman(accel_pitch, gyro_rate_pitch, dt, &kalman_angle_pitch, &kalman_bias_pitch, P_pitch);
    float filtered_roll = updateKalman(accel_roll, gyro_rate_roll, dt, &kalman_angle_roll, &kalman_bias_roll, P_roll);
    
    int xTargetMin = config.invertRoll ? -127 : 127;
    int xTargetMax = config.invertRoll ? 127 : -127;
    int yTargetMin = config.invertPitch ? -127 : 127;
    int yTargetMax = config.invertPitch ? 127 : -127;

    rawXAxis = map(filtered_roll, -config.leanAngle, config.leanAngle, xTargetMin, xTargetMax);
    rawYAxis = map(filtered_pitch, -config.leanAngle, config.leanAngle, yTargetMin, yTargetMax);
    
    int rTargetMin = config.invertRoll ? 0 : 255;
    int rTargetMax = config.invertRoll ? 255 : 0;
    rawRollToRudder = map(filtered_roll, -config.leanAngle, config.leanAngle, rTargetMin, rTargetMax); 
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    timer = micros(); 
  }

  // --- Input Gathering ---
  int potValue = analogRead(THRUST_PIN);
  int mappedPotValue = map(potValue, 0, 1023, 0, 255);
  int finalPotValue = config.isRightHanded ? (255 - mappedPotValue) : mappedPotValue;
  if (config.invertThrottle) finalPotValue = 255 - finalPotValue;

  bool modifierPressed = (readSmart(config.pinModLeft) == LOW || readSmart(config.pinModRight) == LOW);

  // --- LOGIC SWITCHING ---
  if (config.rudderMode == 0) {
    // Mode 0: Pot Swap
    xAxisValue = rawXAxis; 
    if (modifierPressed != lastModifierState) potiControlEngaged = false; 
    if (throttleSafetyLock) potiControlEngaged = false;

    if (potiControlEngaged) {
      if (modifierPressed) {
        rudderValue = finalPotValue;
        storedRudderValue = rudderValue;
      } else {
        thrustValue = finalPotValue;
      }
    } else {
      if (modifierPressed) {
        if (abs(finalPotValue - storedRudderValue) < config.pickupThreshold) {
          potiControlEngaged = true; throttleSafetyLock = false;
        } else {
             digitalWrite(LED_BUILTIN, (millis() % 100 < 50) ? HIGH : LOW);
        }
      } else {
        if (abs(finalPotValue - thrustValue) < config.pickupThreshold) {
          potiControlEngaged = true; throttleSafetyLock = false;
        }
      }
    }
  } else {
    // Mode 1: Roll-to-Yaw
    if (throttleSafetyLock) {
      if (abs(finalPotValue - thrustValue) < config.pickupThreshold) {
        throttleSafetyLock = false; thrustValue = finalPotValue;
      }
    } else {
      thrustValue = finalPotValue; 
    }

    if (modifierPressed) {
      xAxisValue = storedXAxisValue;
      rudderValue = rawRollToRudder;
      storedRudderValue = rudderValue; 
    } else {
      rudderValue = storedRudderValue;
      xAxisValue = rawXAxis;
      storedXAxisValue = xAxisValue;
    }
  }
  
  lastModifierState = modifierPressed;

  // Reset Logic
  if (readSmart(config.pinRst) == LOW || readSmart(config.pinCooliePress) == LOW) {
    rudderValue = 128;
    storedRudderValue = 128;
  }

  // --- OUTPUT & SAFETY ---
  int finalX = constrain(xAxisValue, -127, 127);
  int finalY = constrain(rawYAxis, -127, 127);
  int finalRudder = constrain(rudderValue, 0, 255);

  if (!safetyCutoffActive) {
     if (currentMillis - lastBufferUpdate > 100) { 
       pitchBuffer[bufferIndex] = finalY;
       rudderBuffer[bufferIndex] = finalRudder;
       bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
       lastBufferUpdate = currentMillis;
     }
  }
  
  if (isImuActive) {
    // Use Configurable Angle
    float cutoff = (float)config.safetyAngle;
    if (abs(safety_roll) > cutoff || abs(safety_pitch) > cutoff) safetyCutoffActive = true;
    
    if (safetyCutoffActive && !wasSafetyActive) {
      if (config.smartSafety) {
        safetyHoldPitch = calculateAverage(pitchBuffer, BUFFER_SIZE);
        safetyHoldRudder = calculateAverage(rudderBuffer, BUFFER_SIZE);
      } else {
        safetyHoldPitch = 0;
        safetyHoldRudder = 128;
      }
    }
    
    if (safetyCutoffActive) {
      if (abs(safety_roll) < 15.0 && abs(safety_pitch) < 15.0) safetyCutoffActive = false; 
    }
    
    if (safetyCutoffActive) { 
      finalX = 0; 
      finalY = safetyHoldPitch; 
      finalRudder = safetyHoldRudder; 
    }
    wasSafetyActive = safetyCutoffActive;
  }

  Joystick.setXAxis(finalX);
  Joystick.setYAxis(finalY);
  Joystick.setZAxis(0);  
  Joystick.setThrottle(thrustValue);
  Joystick.setRudder(finalRudder);
  
  int hatAngle = -1;
  if (readSmart(config.pinCoolieUp) == LOW) hatAngle = 0;
  else if (readSmart(config.pinCoolieRight) == LOW) hatAngle = 90;
  else if (readSmart(config.pinCoolieDown) == LOW) hatAngle = 180;
  else if (readSmart(config.pinCoolieLeft) == LOW) hatAngle = 270;
  Joystick.setHatSwitch(0, hatAngle);
  
  Joystick.setButton(0, readSmart(config.pinBtn1) == LOW);
  Joystick.setButton(1, readSmart(config.pinBtn4) == LOW);
  Joystick.setButton(2, readSmart(config.pinCooliePress) == LOW); 
  Joystick.setButton(3, readSmart(config.pinSet) == LOW);
  Joystick.setButton(4, readSmart(config.pinRst) == LOW);
  Joystick.sendState();
}

//================================================================
// HELPERS
//================================================================

void initBuffers() {
  for(int i=0; i<BUFFER_SIZE; i++) {
    pitchBuffer[i] = 0;
    rudderBuffer[i] = 128;
  }
}

int calculateAverage(int* buffer, int size) {
  long sum = 0;
  for(int i=0; i<size; i++) {
    sum += buffer[i];
  }
  return (int)(sum / size);
}

void saveConfiguration() {
  EEPROM.put(0, config);
}

void loadConfiguration() {
  EEPROM.get(0, config);
  if (config.version != CONFIG_VERSION) {
    config.version = CONFIG_VERSION;
    config.leanAngle = 40;
    config.isRightHanded = true;
    config.kalmanLevel = 1;
    config.pickupThreshold = 20;
    config.rudderMode = 0; 
    config.invertRoll = false;
    config.invertPitch = false;
    config.invertThrottle = false;
    config.smartSafety = true;
    config.safetyAngle = 80; // Default Safety Angle
    
    config.pinCoolieUp = 9; config.pinCoolieDown = 8; config.pinCoolieLeft = 7; config.pinCoolieRight = 6;
    config.pinCooliePress = 5; config.pinBtn1 = 15; config.pinBtn4 = 16;
    config.pinModLeft = A3; config.pinModRight = 20; config.pinSet = 4; config.pinRst = 10;
  }
  applyKalmanSettings(config.kalmanLevel);
}

void applyKalmanSettings(int level) {
  switch (level) {
    case 0: Q_angle = 0.001; R_measure = 0.03; break;
    case 1: Q_angle = 0.01; R_measure = 0.01; break;
    case 2: Q_angle = 0.05; R_measure = 0.005; break;
  }
}

//================================================================
// MENU & DEBUG LOGIC
//================================================================
void runSetupMenu() {
  digitalWrite(LED_BUILTIN, HIGH);
  while(digitalRead(ACTIVATION_PIN) == LOW) { delay(10); } 

  bool needsRedraw = true;
  while (true) {
    if (needsRedraw) {
      const char* rudderModeDesc[] = { "Pot Swap (Pickup)", "Roll-to-Yaw (Freeze)" };
      const char* kalmanDesc[] = { "Smooth", "Quick", "Fastest" }; // NEW: Translations
      
      Serial.print("\n\n--- Joystick Setup (v"); Serial.print(FIRMWARE_VERSION); Serial.println(") ---");
      Serial.println("Current Settings:");
      Serial.print("  1. Lean Angle:       "); Serial.println(config.leanAngle);
      Serial.print("  2. Handedness:       "); Serial.println(config.isRightHanded ? "Right" : "Left");
      
      // IMPROVED: Text description for Responsiveness
      Serial.print("  3. Responsiveness:   "); 
      Serial.print(config.kalmanLevel);
      Serial.print(" - ");
      Serial.println(kalmanDesc[config.kalmanLevel]);
      
      Serial.print("  4. Pickup Threshold: "); Serial.println(config.pickupThreshold);
      Serial.print("  5. Rudder Mode:      "); Serial.print(config.rudderMode); Serial.print(" - "); Serial.println(rudderModeDesc[config.rudderMode]);
      Serial.println("  6. Remap Buttons (Learn)");
      Serial.println("  7. Invert Axes"); 
      Serial.println("  8. Test Inputs (Debug)"); 
      Serial.print("  9. Smart Safety:     "); Serial.println(config.smartSafety ? "ON (Avg 3s)" : "OFF (Neutral)");
      
      // NEW: Safety Angle Option
      Serial.print(" 10. Safety Cutoff:    "); Serial.print(config.safetyAngle); Serial.println(" deg");
      
      Serial.println("\nOptions: Type number. Press ACTIVATION BUTTON to Save & Exit.");
      needsRedraw = false;
    }
    
    if (digitalRead(ACTIVATION_PIN) == LOW) {
      saveConfiguration();
      Serial.println("Saved. Restarting...");
      for(int i=0; i<5; i++) { digitalWrite(LED_BUILTIN, LOW); delay(50); digitalWrite(LED_BUILTIN, HIGH); delay(50); }
      return;
    }

    if (Serial.available() > 0) {
      char choice = Serial.read();
      // Handle multi-digit input for "10"
      if (choice == '1') {
         delay(50); // wait for potential second digit
         if (Serial.peek() == '0') {
             Serial.read(); // consume '0'
             choice = 'A'; // Special internal code for 10
         }
      }
      while(Serial.available() > 0) { Serial.read(); }
      
      if (choice == '1') {
        Serial.print("New Angle: "); while (Serial.available() == 0) {} int v = Serial.parseInt();
        if (v >= 20 && v <= 60) config.leanAngle = v;
      } else if (choice == '2') { config.isRightHanded = !config.isRightHanded;
      } else if (choice == '3') { config.kalmanLevel = (config.kalmanLevel + 1) % 3; applyKalmanSettings(config.kalmanLevel);
      } else if (choice == '4') {
        Serial.print("New Thresh: "); while (Serial.available() == 0) {} int v = Serial.parseInt();
        if (v >= 5 && v <= 50) config.pickupThreshold = v;
      } else if (choice == '5') { config.rudderMode = (config.rudderMode + 1) % 2; 
      } else if (choice == '6') { learnButtons(); needsRedraw = true; 
      } else if (choice == '7') { runInvertMenu(); needsRedraw = true; 
      } else if (choice == '8') { runDebugMenu(); needsRedraw = true; 
      } else if (choice == '9') { config.smartSafety = !config.smartSafety; 
      } else if (choice == 'A') { // Option 10
        Serial.print("New Safety Angle (50-89): "); while (Serial.available() == 0) {} int v = Serial.parseInt();
        if (v >= 50 && v <= 89) config.safetyAngle = v;
      }
      
      needsRedraw = true;
    }
    delay(100);
  }
}

void runDebugMenu() {
  Serial.println("\n--- INPUT TEST MODE ---");
  Serial.println("Press keys to test. Press ACTIVATION BUTTON to exit.");
  delay(1000);
  while(digitalRead(ACTIVATION_PIN) == HIGH) {
    int ml = readSmart(config.pinModLeft);
    int mr = readSmart(config.pinModRight);
    Serial.print("MODS: L["); Serial.print(ml == LOW ? "X" : " "); Serial.print("] R["); Serial.print(mr == LOW ? "X" : " "); Serial.print("] ");
    Serial.print("BTN: 1["); Serial.print(readSmart(config.pinBtn1)==LOW ? "X":" "); 
    Serial.print("] 4["); Serial.print(readSmart(config.pinBtn4)==LOW ? "X":" ");
    Serial.print("] RST["); Serial.print(readSmart(config.pinRst)==LOW ? "X":" ");
    Serial.print("] SET["); Serial.print(readSmart(config.pinSet)==LOW ? "X":" ");
    Serial.print("] ");
    Serial.print("HAT: U["); Serial.print(readSmart(config.pinCoolieUp)==LOW?"X":" ");
    Serial.print("] D["); Serial.print(readSmart(config.pinCoolieDown)==LOW?"X":" ");
    Serial.print("] L["); Serial.print(readSmart(config.pinCoolieLeft)==LOW?"X":" ");
    Serial.print("] R["); Serial.print(readSmart(config.pinCoolieRight)==LOW?"X":" ");
    Serial.print("] P["); Serial.print(readSmart(config.pinCooliePress)==LOW?"X":" ");
    Serial.print("] ");
    int pot = analogRead(THRUST_PIN);
    Serial.print("POT: "); Serial.println(pot);
    delay(100);
  }
}

void runInvertMenu() {
  bool subRedraw = true;
  while(true) {
    if (subRedraw) {
      Serial.println("\n--- INVERT AXES ---");
      Serial.print("  1. Invert Roll (X):  "); Serial.println(config.invertRoll ? "YES" : "NO");
      Serial.print("  2. Invert Pitch (Y): "); Serial.println(config.invertPitch ? "YES" : "NO");
      Serial.print("  3. Invert Throttle:  "); Serial.println(config.invertThrottle ? "YES" : "NO");
      Serial.println("  4. [BACK TO MAIN MENU]");
      subRedraw = false;
    }
    if (Serial.available() > 0) {
      char c = Serial.read();
      while(Serial.available() > 0) Serial.read();
      if (c == '1') config.invertRoll = !config.invertRoll;
      else if (c == '2') config.invertPitch = !config.invertPitch;
      else if (c == '3') config.invertThrottle = !config.invertThrottle;
      else if (c == '4') return;
      subRedraw = true;
    }
    delay(100);
  }
}

uint8_t waitForButtonPress(const char* btnName) {
  Serial.print("\n> Press Button for: "); Serial.print(btnName); Serial.println(" (or 's' to Skip)");
  delay(500); 
  while(true) {
    if (digitalRead(ACTIVATION_PIN) == LOW) return 0; 
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S') { Serial.println("   -> Skipped"); return PIN_DISABLED; }
    }
    for (int i = 0; i < numCandidates; i++) {
      int pin = candidatePins[i];
      pinMode(pin, INPUT_PULLUP); 
      if (digitalRead(pin) == LOW) {
        Serial.print("   -> Pin "); Serial.println(pin);
        digitalWrite(LED_BUILTIN, LOW); delay(200); digitalWrite(LED_BUILTIN, HIGH);
        while(digitalRead(pin) == LOW) { delay(10); } 
        return pin;
      }
    }
  }
}

void learnButtons() {
  Serial.println("\n--- LEARN MODE: Press Buttons / 's' to Skip ---");
  uint8_t p;
  p = waitForButtonPress("COOLIE UP");    if(p != 0) config.pinCoolieUp = p;
  p = waitForButtonPress("COOLIE RIGHT"); if(p != 0) config.pinCoolieRight = p;
  p = waitForButtonPress("COOLIE DOWN");  if(p != 0) config.pinCoolieDown = p;
  p = waitForButtonPress("COOLIE LEFT");  if(p != 0) config.pinCoolieLeft = p;
  p = waitForButtonPress("COOLIE PRESS"); if(p != 0) config.pinCooliePress = p;
  p = waitForButtonPress("BUTTON 1 (Top)"); if(p != 0) config.pinBtn1 = p;
  p = waitForButtonPress("BUTTON 4 (Side)"); if(p != 0) config.pinBtn4 = p;
  p = waitForButtonPress("LEFT THUMB (Mod 1)"); if(p != 0) config.pinModLeft = p;
  p = waitForButtonPress("RIGHT THUMB (Mod 2)"); if(p != 0) config.pinModRight = p;
  p = waitForButtonPress("SET Button"); if(p != 0) config.pinSet = p;
  p = waitForButtonPress("RESET Button"); if(p != 0) config.pinRst = p;
  Serial.println("\n--- COMPLETE ---");
  delay(1000);
}

float updateKalman(float newAngle, float newRate, float dt, float* kalman_angle, float* kalman_bias, float P[2][2]) {
  float rate = newRate - *kalman_bias;
  *kalman_angle += dt * rate;
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  float S = P[0][0] + R_measure;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
  float y = newAngle - *kalman_angle;
  *kalman_angle += K[0] * y;
  *kalman_bias += K[1] * y;
  float P00_temp = P[0][0]; float P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp; P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp; P[1][1] -= K[1] * P01_temp;
  return *kalman_angle;
}