// ESP32-S3 WROOM-1 N16R8 Controller for Pan & Tilt test
#include <AccelStepper.h>

// Stepper pins
#define step1Pin 10      // GPIO 9  - Pan STEP
#define dir1Pin  11     // GPIO 10 - Pan DIR
#define step2Pin 12    // GPIO 11 - Tilt STEP
#define dir2Pin  13     // GPIO 12 - Tilt DIR

#define tiltLimitSwitch  18
#define panLimitSwitchL  16
#define panLimitSwitchR  17

long leftSteps = 0;
long rightSteps = 0;
long max_pos = 0;
long centerPanPosition = 0;
bool panHomingDone = false;

bool centeredLeft = false;
bool centeredRight = false;

long maxTiltPos = 45000;
long tiltCurrentPosition = 0;

AccelStepper stepperPan(AccelStepper::DRIVER, step1Pin, dir1Pin);
AccelStepper stepperTilt(AccelStepper::DRIVER, step2Pin, dir2Pin);


void setup() {
  pinMode(panLimitSwitchL, INPUT_PULLUP);
  pinMode(panLimitSwitchR, INPUT_PULLUP);
  pinMode(tiltLimitSwitch, INPUT_PULLUP);
  Serial.begin(115200);

  pinMode(RGB_BUILTIN, OUTPUT);
  for(int i = 0; i<10; i++){
    neopixelWrite(RGB_BUILTIN, 100, 100, 100);
    delay(70);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    delay(70);
    neopixelWrite(RGB_BUILTIN, 100, 100, 100);
    delay(70);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    delay(300);
  }


  stepperPan.setMaxSpeed(5000);
  stepperPan.setAcceleration(8000);
  stepperTilt.setMaxSpeed(8000);
  stepperTilt.setAcceleration(8000);

  //Start pan homing
  neopixelWrite(RGB_BUILTIN, 255, 0, 0);
  stepperPan.setSpeed(-3000);
  while (digitalRead(panLimitSwitchL) == HIGH) {
  stepperPan.runSpeed();
  }
  // Set left as zero
  stepperPan.setCurrentPosition(0);
  leftSteps = 0;
  Serial.println("Left homed");
  delay(250);
  // Move RIGHT until right switch is pressed
  neopixelWrite(RGB_BUILTIN, 0, 255, 0);
  stepperPan.setSpeed(3000);
  while (digitalRead(panLimitSwitchR) == HIGH) {
    stepperPan.runSpeed();
  }

  // Record max position
  rightSteps = stepperPan.currentPosition();
  max_pos = rightSteps;

  Serial.print("Right Steps: ");
  Serial.println(rightSteps);

  // Go to center
  centerPanPosition = max_pos / 2;
  stepperPan.moveTo(centerPanPosition);

  // Move to center
  while (stepperPan.distanceToGo() != 0) {
    stepperPan.run();
  }

  Serial.print("Center Pan Position: ");
  Serial.println(centerPanPosition);
  panHomingDone = true;


  //tilt homing 
  neopixelWrite(RGB_BUILTIN, 0, 0, 255);
  stepperTilt.setSpeed(-4500);
  while (digitalRead(tiltLimitSwitch) == HIGH) {
    stepperTilt.runSpeed();
  }
  stepperTilt.setCurrentPosition(0);

  stepperTilt.moveTo(maxTiltPos/2);
  while (stepperTilt.distanceToGo() != 0) {
    stepperTilt.run();
  }

  Serial.println("Tilt homed.");
  Serial.println("First test done.");

  pinMode(RGB_BUILTIN, OUTPUT);
  for(int i = 0; i<7; i++){
    neopixelWrite(RGB_BUILTIN, 100, 100, 100);
    delay(70);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    delay(70);
    neopixelWrite(RGB_BUILTIN, 100, 100, 100);
    delay(70);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    delay(300);
  }
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);


}

void loop() {

  /*
  stepperPan.moveTo(1000);
  stepperTilt.moveTo(500);
  while (stepperTilt.distanceToGo() != 0 || stepperPan.distanceToGo() != 0) {
    stepperPan.run();
    stepperTilt.run();
  }
  delay(200);
  stepperPan.moveTo(max_pos-1000);
  stepperTilt.moveTo(maxTiltPos - 1000);
  while (stepperTilt.distanceToGo() != 0 || stepperPan.distanceToGo() != 0) {
    stepperPan.run();
    stepperTilt.run();
  }
  delay(200);
  */
}
