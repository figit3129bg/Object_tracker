// the controller for this sketch is an ESP32 S3 WROOM 1 N16R8

// all of the pins bellow are pins that have ADC and are used for stepper control
// MS1, MS2 and MS3 pins are all set to HIGH and its hardwired into the solder board.
// All of them are conected to 5V

#include <AccelStepper.h>

#define step1Pin 9      //GPIO 9
#define dir1Pin 10      //GPIO 10
#define step2Pin 11     //GPIO 11
#define dir2Pin 12      //GPIO 12

// Centering pins are 16, 17 and 18. 
#define tiltLimitSwitch 16
#define panLimitSwitchL 17
#define panLimitSwitchR 18

long maxPanPos = 0;
long maxTiltPos = 50000;
long leftSteps = 0;
long rightSteps = 0;
long panCenterPosition = 0;
long tiltCurentPosition = 0;

float maxSpeed = 5000; //max speed in steps

AccelStepper stepperPan(AccelStepper::DRIVER, step1Pin, dir1Pin);
AccelStepper stepperTilt(AccelStepper::DRIVER, step2Pin, dir2Pin);

void setup() {
  Serial.begin(115200);

  pinMode(panLimitSwitchL, INPUT_PULLUP);
  pinMode(panLimitSwitchR, INPUT_PULLUP);
  pinMode(tiltLimitSwitch, INPUT_PULLUP);
  
  stepperPan.setMaxSpeed(5000);
  stepperPan.setAcceleration(8000);
  stepperTilt.setMaxSpeed(5000);
  stepperTilt.setAcceleration(8000);

  // -----Homing-----
  // Rotate left until switch pressed
  stepperPan.setSpeed(-3000);
  while (digitalRead(panLimitSwitchL) == HIGH) stepperPan.runSpeed();
  steppePan.setCurrentPosition(0);
  leftSteps = 0;
  Serial.println("Left homed");
  delay(250);

  // Rotate right until switxch pressed
  stepperPan.setSpeed(3000);
  while (digitalRead(panLimitSwitchR) == HIGH) stepperPan.runSpeed();

  // Record max pan position
  rightSteps = stepperPan.currentPosition();
  maxPanPos = rightSteps;
  Serial.print("Right Steps: ");
  Serial.println(rightSteps);      
      
  // Centering the Z axis
  panCenterPosition = maxPanPos/2;
  stepperPan.moveTo(panCenterPosition);
  stepperPan.runToPosition();

  // Tilt homing 
  stepperTilt.setSPeed(-3000);
  while (digitalRead(tiltLimitSwitch) == HIGH) stepperTilt.runSpeed();
  stepperTilt.setCurrentPosition(0);    

}

void loop() {
  // put your main code here, to run repeatedly:

}
