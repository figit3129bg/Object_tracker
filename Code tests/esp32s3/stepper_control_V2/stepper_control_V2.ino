// ESP32-S3 WROOM-1 N16R8 Controller for Pan & Tilt with RGB feedback

#include <AccelStepper.h>

// Stepper pins
#define step1Pin 9      // GPIO 9  - Pan STEP
#define dir1Pin  10     // GPIO 10 - Pan DIR
#define step2Pin 11     // GPIO 11 - Tilt STEP
#define dir2Pin  12     // GPIO 12 - Tilt DIR

// Limit switches (active LOW with INPUT_PULLUP)
#define tiltLimitSwitch  16
#define panLimitSwitchL  17
#define panLimitSwitchR  18

// Built-in RGB LED (usually GPIO 48 on ESP32-S3 dev boards) mine is on 38
#define RGB_BUILTIN 38

// Control parameters
#define DEADZONE 20

float currentMaxSpeed = 1000;  // Steps per second - adjustable

// Virtual joystick inputs from phone (0–1000, neutral 500)
int virtualPanInput  = 500;
int virtualTiltInput = 500;

// Position & homing variables
long maxPanPos         = 0;
long maxTiltPos        = 50000;
long leftSteps         = 0;
long rightSteps        = 0;
long panCenterPosition = 0;
long tiltCurrentPosition = 0; 

// Stepper objects
AccelStepper stepperPan (AccelStepper::DRIVER, step1Pin, dir1Pin);
AccelStepper stepperTilt(AccelStepper::DRIVER, step2Pin, dir2Pin);

// Homing state flags
bool homingTiltActive  = false;
bool homingPanActive   = false;
bool homingBothActive  = false;

// RGB blinking state
bool isHomingTilt  = false;
bool isHomingPan   = false;
bool isHomingBoth  = false;

unsigned long previousBlinkMillis = 0;
int blinkCount = 0;
bool ledOn = false;

const unsigned long fastBlinkInterval   = 150;   // ON/OFF time for fast blink (ms)
const unsigned long pauseAfterSequence  = 600;   // Pause after each blink sequence

const int tiltPattern[] = {2, 0};   // 2 blinks then pause
const int panPattern[]  = {3, 0};   // 3 blinks then pause

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("ESP32-S3 ready - Starting initial homing...");

  // Limit switches
  pinMode(panLimitSwitchL, INPUT_PULLUP);
  pinMode(panLimitSwitchR, INPUT_PULLUP);
  pinMode(tiltLimitSwitch, INPUT_PULLUP);

  // RGB LED
  pinMode(RGB_BUILTIN, OUTPUT);
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);  // Off at start

  // Stepper setup
  stepperPan.setMaxSpeed(currentMaxSpeed);
  stepperPan.setAcceleration(8000);
  stepperTilt.setMaxSpeed(currentMaxSpeed);
  stepperTilt.setAcceleration(8000);

  // Initial homing typicaly it is blocking which is not good but in setup it isnt a problem
  startPanHoming();
  startTiltHoming();

  Serial.println("ESP32-S3 ready - Waiting for commands");
}

void loop() {
  // Read commands from phone
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("PAN:")) {
      virtualPanInput = cmd.substring(4).toInt();
      virtualPanInput = constrain(virtualPanInput, 0, 1000);
      Serial.print("Phone input PAN: ");
      Serial.println(virtualPanInput);
    }
    else if (cmd.startsWith("TILT:")) {
      virtualTiltInput = cmd.substring(5).toInt();
      virtualTiltInput = constrain(virtualTiltInput, 0, 1000);
      Serial.print("Phone input TILT: ");
      Serial.println(virtualTiltInput);
    }
    else if (cmd.startsWith("Homing:")) {   // 1 - tilt, 2 - pan, 3 - both
      int homingType = cmd.substring(7).toInt();
      if (homingType == 1) {
        startTiltHoming();
      }
      else if (homingType == 2) {
        startPanHoming();
      }
      else if (homingType == 3) {
        startBothHoming();
      }
    }
  }

  // Update motor control
  updatePanMotor();
  updateTiltMotor();

  // Handle non-blocking homing
  handleHoming();

  // Always update RGB blinking
  updateRGBBlink();
}

// ==================== HOMING CONTROL ====================

void startTiltHoming() {
  homingTiltActive = true;
  isHomingTilt = true;
  stepperTilt.setSpeed(-3000);  // Toward limit switch
  Serial.println("Tilt homing started");
}

void startPanHoming() {
  homingPanActive = true;
  isHomingPan = true;
  stepperPan.setSpeed(-3000);   // Toward left limit
  Serial.println("Pan homing started");
}

void startBothHoming() {
  homingBothActive = true;
  isHomingTilt = true;
  isHomingPan = true;
  stepperTilt.setSpeed(-3000);
  stepperPan.setSpeed(-3000);
  Serial.println("Both homing started");
}

void handleHoming() {
  // Tilt homing
  if (homingTiltActive) {
    if (digitalRead(tiltLimitSwitch) == HIGH) {
      stepperTilt.runSpeed();
    } else {
      stepperTilt.setCurrentPosition(0);
      homingTiltActive = false;
      isHomingTilt = false;
      Serial.println("Tilt homed");
    }
  }

  // Pan homing (two-phase: left -> right -> center)
  if (homingPanActive) {
    if (digitalRead(panLimitSwitchL) == HIGH) {
      stepperPan.runSpeed();
    } else {
      stepperPan.setCurrentPosition(0);
      leftSteps = 0;
      Serial.println("Pan left homed");

      // Phase 2: Move right until right switch
      stepperPan.setSpeed(3000);
      if (digitalRead(panLimitSwitchR) == HIGH) {
        stepperPan.runSpeed();
      } else {
        rightSteps = stepperPan.currentPosition();
        maxPanPos = rightSteps;
        Serial.print("Right Steps: ");
        Serial.println(rightSteps);

        // Phase 3: Center
        panCenterPosition = maxPanPos / 2;
        stepperPan.moveTo(panCenterPosition);
        // Non-blocking move to center
        if (stepperPan.distanceToGo() != 0) {
          stepperPan.run();
        } else {
          homingPanActive = false;
          isHomingPan = false;
          Serial.println("Pan centered");
        }
      }
    }
  }
}

// ==================== RGB BLINKING (NON-BLOCKING) ====================

void updateRGBBlink() {
  unsigned long currentMillis = millis();

  if (!isHomingTilt && !isHomingPan && !isHomingBoth) {
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    blinkCount = 0;
    ledOn = false;
    return;
  }

  // Choose pattern
  const int* pattern = (isHomingTilt && !isHomingBoth) ? tiltPattern :
                       (isHomingPan  && !isHomingBoth) ? panPattern  :
                       tiltPattern;  // Default for both

  unsigned long interval = ledOn ? fastBlinkInterval : pauseAfterSequence;

  if (currentMillis - previousBlinkMillis >= interval) {
    previousBlinkMillis = currentMillis;

    if (ledOn) {
      // Turn OFF
      neopixelWrite(RGB_BUILTIN, 0, 0, 0);
      ledOn = false;
      blinkCount++;
    } else {
      // Turn ON if still in sequence
      if (pattern[blinkCount] > 0) {
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);  // Red
        ledOn = true;
      } else {
        // Sequence done → reset
        blinkCount = 0;
      }
    }
  }
}

// ==================== MOTOR CONTROL ====================

void updatePanMotor() {
  int center = 500;
  int delta = virtualPanInput - center;
  long currentPos = stepperPan.currentPosition();

  if (abs(delta) < DEADZONE) {
    stepperPan.setSpeed(0);
    return;
  }

  float speed = map(abs(delta), DEADZONE, 500, 0, currentMaxSpeed);

  // Limit switch protection
  if (digitalRead(panLimitSwitchL) == LOW && delta < 0) {
    stepperPan.setSpeed(0);
    return;
  }
  if (digitalRead(panLimitSwitchR) == LOW && delta > 0) {
    stepperPan.setSpeed(0);
    return;
  }

  if (delta > 0 && (maxPanPos == 0 || currentPos < maxPanPos)) {
    stepperPan.setSpeed(speed);    // Right
  } else if (delta < 0 && currentPos > 0) {
    stepperPan.setSpeed(-speed);   // Left
  } else {
    stepperPan.setSpeed(0);
  }

  stepperPan.runSpeed();
}

void updateTiltMotor() {
  int center = 500;
  int delta = virtualTiltInput - center;
  tiltCurrentPosition = stepperTilt.currentPosition();

  if (abs(delta) < DEADZONE) {
    stepperTilt.setSpeed(0);
    return;
  }

  float speed = map(abs(delta), DEADZONE, 500, 0, currentMaxSpeed);

  // Bottom limit + virtual upper limit
  if (digitalRead(tiltLimitSwitch) == LOW && delta < 0) {
    stepperTilt.setSpeed(0);
    return;
  }

  if (delta > 0 && tiltCurrentPosition < maxTiltPos) {
    stepperTilt.setSpeed(speed);    // Up
  } else if (delta < 0 && tiltCurrentPosition > 0) {
    stepperTilt.setSpeed(-speed);   // Down
  } else {
    stepperTilt.setSpeed(0);
  }

  stepperTilt.runSpeed();
}