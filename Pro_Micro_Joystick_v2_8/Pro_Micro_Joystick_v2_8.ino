// Custom Joystick Firmware
// VERSION 2.8: Reverted rudder control to "pickup" logic.
// - Removed automatic rudder centering.
// - Removed rudder calibration from the setup menu.

#include "Joystick.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>

//================================================================
// CONFIGURATION STRUCTURE & DEFAULTS
//================================================================
const String FIRMWARE_VERSION = "2.8";

// MODIFIED: Removed rudderMidpoint from the struct
struct JoystickConfig {
  int version;
  int leanAngle;
  bool isRightHanded;
  int kalmanLevel;
};

JoystickConfig config;
const int CONFIG_VERSION = 4; // NEW: Incremented due to struct change

//================================================================
// PIN DEFINITIONS
//================================================================
const int THRUST_PIN        = A0;
const int COOLIE_UP_PIN     = 9;
const int COOLIE_DOWN_PIN   = 8;
const int COOLIE_LEFT_PIN   = 7;
const int COOLIE_RIGHT_PIN  = 6;
const int COOLIE_PRESS_PIN  = 5;
const int SET_BUTTON_PIN    = 4;
const int RST_BUTTON_PIN    = 10;
const int BUTTON_1_PIN      = 15;
const int BUTTON_2_PIN      = A3; // Rudder Modifier
const int BUTTON_3_PIN      = 20; // Rudder Modifier
const int BUTTON_4_PIN      = 16;
const int ACTIVATION_PIN    = 14;

//================================================================
// SENSOR OBJECT & STATE
//================================================================
Adafruit_MPU6050 mpu;
bool isImuActive = false;
int lastActivationButtonState = HIGH;
int yawValue = 0;
int thrustValue = 0;
int propellerValue = 128; // Initial value before first use
bool lastPropellerMode = false;
bool potiControlEngaged = true;
const int PICKUP_THRESHOLD = 20;
bool yawGestureActive = false;
float accel_y_offset = 0;
bool isCalibrated = false;
const float GESTURE_THRESHOLD = 3.0;
const int GESTURE_INCREMENT = 5;

//================================================================
// KALMAN FILTER SETUP
//================================================================
float kalman_angle_pitch = 0, kalman_unbiased_rate_pitch = 0, kalman_bias_pitch = 0;
float P_pitch[2][2] = { { 1, 0 }, { 0, 1 } };
float kalman_angle_roll = 0, kalman_unbiased_rate_roll = 0, kalman_bias_roll = 0;
float P_roll[2][2] = { { 1, 0 }, { 0, 1 } };
float Q_angle, Q_bias = 0.003, R_measure;
unsigned long timer;

void loadConfiguration();
void applyKalmanSettings(int level);
void runSetupMenu();
float updateKalman(float newAngle, float newRate, float dt, float* kalman_angle, float* kalman_bias, float P[2][2]);
void calibrateYawGesture();

//================================================================
// SETUP
//================================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(ACTIVATION_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  
  loadConfiguration();

  if (digitalRead(ACTIVATION_PIN) == LOW) {
    runSetupMenu();
  }

  Wire.begin();
  if (!mpu.begin()) {
    while (1) { delay(10); }
  }
  timer = micros();

  pinMode(THRUST_PIN, INPUT);
  pinMode(COOLIE_UP_PIN, INPUT_PULLUP);
  pinMode(COOLIE_DOWN_PIN, INPUT_PULLUP);
  pinMode(COOLIE_LEFT_PIN, INPUT_PULLUP);
  pinMode(COOLIE_RIGHT_PIN, INPUT_PULLUP);
  pinMode(COOLIE_PRESS_PIN, INPUT_PULLUP);
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  pinMode(BUTTON_4_PIN, INPUT_PULLUP);
  pinMode(SET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RST_BUTTON_PIN, INPUT_PULLUP);
  
  Joystick.begin(false);
}

//================================================================
// MAIN LOOP
//================================================================
void loop() {
  int currentActivationButtonState = digitalRead(ACTIVATION_PIN);
  if (lastActivationButtonState == HIGH && currentActivationButtonState == LOW) {
    delay(50);
    isImuActive = !isImuActive;
  }
  lastActivationButtonState = currentActivationButtonState;

  bool button2_pressed = (digitalRead(BUTTON_2_PIN) == LOW);
  bool button3_pressed = (digitalRead(BUTTON_3_PIN) == LOW);
  bool propellerMode = (button2_pressed || button3_pressed);

  if (propellerMode != lastPropellerMode) {
    potiControlEngaged = false;
  }
  
  int xAxisValue = 0, yAxisValue = 0;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (isImuActive) {
    if (!isCalibrated) { calibrateYawGesture(); }
    digitalWrite(LED_BUILTIN, potiControlEngaged ? HIGH : (millis() % 500 < 250));
    
    float dt = (float)(micros() - timer) / 1000000.0;
    timer = micros();
    float accel_pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
    float accel_roll = atan2(a.acceleration.y, -a.acceleration.z) * 180 / M_PI;
    float gyro_rate_pitch = g.gyro.y * 180 / M_PI;
    float gyro_rate_roll = g.gyro.x * 180 / M_PI;
    float filtered_pitch = updateKalman(accel_pitch, gyro_rate_pitch, dt, &kalman_angle_pitch, &kalman_bias_pitch, P_pitch);
    float filtered_roll = updateKalman(accel_roll, gyro_rate_roll, dt, &kalman_angle_roll, &kalman_bias_roll, P_roll);
    
    xAxisValue = map(filtered_roll, -config.leanAngle, config.leanAngle, 127, -127);
    yAxisValue = map(filtered_pitch, -config.leanAngle, config.leanAngle, 127, -127);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    isCalibrated = false;
  }
  
  int potValue = analogRead(THRUST_PIN);
  int mappedPotValue = map(potValue, 0, 1023, 0, 255);
  int finalPotValue;

  if (config.isRightHanded) {
    finalPotValue = 255 - mappedPotValue;
  } else {
    finalPotValue = mappedPotValue;
  }

  if (potiControlEngaged) {
    if (propellerMode) {
      propellerValue = finalPotValue;
    } else {
      thrustValue = finalPotValue;
    }
  } else {
    if (propellerMode ? (abs(finalPotValue - propellerValue) < PICKUP_THRESHOLD) : (abs(finalPotValue - thrustValue) < PICKUP_THRESHOLD)) {
      potiControlEngaged = true;
    }
  }

  // REMOVED: The automatic rudder centering logic was here.
  // By removing it, the propellerValue now holds its last value when the modifier is released.

  float calibrated_ay = a.acceleration.y - accel_y_offset;
  if (calibrated_ay > GESTURE_THRESHOLD && !yawGestureActive) {
    yawValue -= GESTURE_INCREMENT; yawGestureActive = true;
  } else if (calibrated_ay < -GESTURE_THRESHOLD && !yawGestureActive) {
    yawValue += GESTURE_INCREMENT; yawGestureActive = true;
  } else if (abs(calibrated_ay) < GESTURE_THRESHOLD * 0.8) {
    yawGestureActive = false;
  }
  
  xAxisValue = constrain(xAxisValue, -127, 127);
  yAxisValue = constrain(yAxisValue, -127, 127);
  yawValue = constrain(yawValue, -127, 127);

  bool btn1_state = (digitalRead(BUTTON_1_PIN) == LOW);
  bool btn4_state = (digitalRead(BUTTON_4_PIN) == LOW);
  bool coolie_press_state = (digitalRead(COOLIE_PRESS_PIN) == LOW);
  bool set_btn_state = (digitalRead(SET_BUTTON_PIN) == LOW);
  bool rst_btn_state = (digitalRead(RST_BUTTON_PIN) == LOW);

  int hatAngle = -1;
  if (digitalRead(COOLIE_UP_PIN) == LOW) hatAngle = 0;
  else if (digitalRead(COOLIE_RIGHT_PIN) == LOW) hatAngle = 90;
  else if (digitalRead(COOLIE_DOWN_PIN) == LOW) hatAngle = 180;
  else if (digitalRead(COOLIE_LEFT_PIN) == LOW) hatAngle = 270;

  Joystick.setXAxis(xAxisValue);
  Joystick.setYAxis(yAxisValue);
  Joystick.setZAxis(yawValue);
  Joystick.setThrottle(thrustValue);
  Joystick.setRudder(propellerValue);
  Joystick.setHatSwitch(0, hatAngle);
  Joystick.setButton(0, btn1_state);
  Joystick.setButton(1, btn4_state);
  Joystick.setButton(2, coolie_press_state);
  Joystick.setButton(3, set_btn_state);
  Joystick.setButton(4, rst_btn_state);
  Joystick.sendState();

  lastPropellerMode = propellerMode;
}

//================================================================
// HELPER FUNCTIONS FOR CONFIGURATION
//================================================================
void loadConfiguration() {
  EEPROM.get(0, config);
  // MODIFIED: Check for new config version and set defaults without rudder midpoint
  if (config.version != CONFIG_VERSION) {
    config.version = CONFIG_VERSION;
    config.leanAngle = 40;
    config.isRightHanded = true;
    config.kalmanLevel = 1;
  }
  applyKalmanSettings(config.kalmanLevel);
}

void applyKalmanSettings(int level) {
  switch (level) {
    case 0: Q_angle = 0.001; R_measure = 0.03; break; // Smooth
    case 1: Q_angle = 0.01; R_measure = 0.01; break;  // Quick
    case 2: Q_angle = 0.05; R_measure = 0.005; break; // Fastest
  }
}

void runSetupMenu() {
  digitalWrite(LED_BUILTIN, HIGH);
  while(digitalRead(ACTIVATION_PIN) == LOW) { delay(10); }

  bool needsRedraw = true;

  while (true) {
    if (needsRedraw) {
      const char* kalmanModes[] = {"Smooth", "Quick", "Fastest"};
      
      Serial.print("\n\n--- Joystick Setup Menu (v");
      Serial.print(FIRMWARE_VERSION);
      Serial.println(") ---");
      Serial.println("Current Settings:");
      Serial.print("  1. Max Lean Angle: "); Serial.println(config.leanAngle);
      Serial.print("  2. Handedness:     "); Serial.println(config.isRightHanded ? "Right-Handed (Inverted)" : "Left-Handed (Normal)");
      Serial.print("  3. Responsiveness: "); Serial.println(kalmanModes[config.kalmanLevel]);
      // REMOVED: Rudder midpoint display from menu
      Serial.println("\nOptions:");
      Serial.println("  Type a number to change a setting.");
      Serial.println("  Press the [Activation Button] to save and exit.");
      Serial.println("---------------------------");

      needsRedraw = false;
    }

    if (digitalRead(ACTIVATION_PIN) == LOW) {
      EEPROM.put(0, config);
      Serial.println("\nConfiguration saved! Restarting joystick...");
      for(int i=0; i<5; i++) {
        digitalWrite(LED_BUILTIN, LOW); delay(100);
        digitalWrite(LED_BUILTIN, HIGH); delay(100);
      }
      return;
    }

    if (Serial.available() > 0) {
      char choice = Serial.read();
      while(Serial.available() > 0) { Serial.read(); }

      if (choice == '1') {
        Serial.println("\n> Enter new lean angle (20-60 degrees):");
        while (Serial.available() == 0) {}
        int newAngle = Serial.parseInt();
        if (newAngle >= 20 && newAngle <= 60) {
          config.leanAngle = newAngle;
          Serial.println("--> Lean angle updated.");
        } else {
          Serial.println("--> Invalid angle. Please try again.");
        }
        needsRedraw = true;
      } else if (choice == '2') {
        config.isRightHanded = !config.isRightHanded;
        needsRedraw = true;
      } else if (choice == '3') {
        config.kalmanLevel = (config.kalmanLevel + 1) % 3;
        applyKalmanSettings(config.kalmanLevel);
        needsRedraw = true;
      }
      // REMOVED: Logic for menu item '4'
    }
    delay(100);
  }
}

//================================================================
// ORIGINAL HELPER FUNCTIONS
//================================================================
void calibrateYawGesture() {
  float y_sum = 0;
  int samples = 200;
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(20);
    digitalWrite(LED_BUILTIN, LOW); delay(20);
  }
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    y_sum += a.acceleration.y;
    delay(5);
  }
  accel_y_offset = y_sum / samples;
  isCalibrated = true;
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
  
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  
  return *kalman_angle;
}