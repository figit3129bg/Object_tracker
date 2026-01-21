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

#define deadzone 20
float currentMaxSpeed = 1000; // adjustable
// float maxSpeed = 5000; //max speed in steps hardwired

// Virtual inputs from phone (default to neutral)
int virtualPanInput = 500;      // 0–1000 neutral is 500
int virtualTiltInput = 500;     // 0–1000 neutral is 500

long maxPanPos = 0;
long maxTiltPos = 50000;
long leftSteps = 0;
long rightSteps = 0;
long panCenterPosition = 0;
long tiltCurentPosition = 0;



AccelStepper stepperPan(AccelStepper::DRIVER, step1Pin, dir1Pin);
AccelStepper stepperTilt(AccelStepper::DRIVER, step2Pin, dir2Pin);

void setup() {
  Serial.begin(115200);
  while (!Serial) {delay(10);} 
  Serial.println("ESP32-S3 ready - Homing");

  pinMode(panLimitSwitchL, INPUT_PULLUP);
  pinMode(panLimitSwitchR, INPUT_PULLUP);
  pinMode(tiltLimitSwitch, INPUT_PULLUP);
  
  stepperPan.setMaxSpeed(currentMaxSpeed);
  stepperPan.setAcceleration(8000);    // Smooth start/stop; adjust
  stepperTilt.setMaxSpeed(currentMaxSpeed);
  stepperTilt.setAcceleration(8000);

  // -----Homing-----
  // Rotate left until switch pressed
  stepperPan.setSpeed(-3000);
  while (digitalRead(panLimitSwitchL) == HIGH) {stepperPan.runSpeed();}
  stepperPan.setCurrentPosition(0);
  leftSteps = 0;
  Serial.println("Left homed");
  delay(250);

  // Rotate right until switxch pressed
  stepperPan.setSpeed(3000);
  while (digitalRead(panLimitSwitchR) == HIGH) {stepperPan.runSpeed();}

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
  stepperTilt.setSpeed(-3000);
  while (digitalRead(tiltLimitSwitch) == HIGH) {stepperTilt.runSpeed();}
  stepperTilt.setCurrentPosition(0);    

  Serial.println("ESP32-S3 ready - Waiting for comands");
}

// This code is supposed to run with an Android test app

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if(cmd.startsWith("Pan:")){
      virtualPanInput = cmd.substring(4).toInt();
      virtualPanInput = constrain(virtualPanInput, 0, 1000); // Limiting the input number to be withing limits
      Serial.print("Phone input PAN: ");
      Serial.println(virtualPanInput);
    }
    else if (cmd.startsWith("TILT:")) {
      virtualTiltInput = cmd.substring(5).toInt();
      virtualTiltInput = constrain(virtualTiltInput, 0, 1000);  
      Serial.print("Phone input TILT: ");
      Serial.println(virtualTiltInput);
    }
    // Will implement an rehoming comand
  }

}

void updatePanMotor(){
  int center = 500;
  int delta = virtualPanInput - center;

  long currentPos = stepperPan.currentPosition();

  if(abs(delta)<20){
    stepperPan.setSpeed(0);
    return;
  }

  float speed = map(abs(delta), deadzone, 500, 0, currentMaxSpeed);

  // Checks that override the inout if the limit switches are pressed for rpotecting the motor and the system
  if (digitalRead(panLimitSwitchL) == LOW && delta < 0) {
    stepperPan.setSpeed(0);
    return;
  }
  if (digitalRead(panLimitSwitchR) == LOW && delta > 0) {
    stepperPan.setSpeed(0);
    return;
  }
  if (delta > 0 && (maxPanPos == 0 || currentPos < maxPanPos)) {  
    stepperPan.setSpeed(speed);                // Positive - right
  }else if (delta < 0 && currentPos > 0) {
    stepperPan.setSpeed(-speed);               // Negative - left
  } else {
    stepperPan.setSpeed(0);
  }

  stepperPan.runSpeed();
}

void updateTiltMotor(){
  int center = 500;
  int delta = virtualTiltInput - center;

  tiltCurentPosition = stepperTilt.currentPosition();

  if (abs(delta) < deadzone) {
    stepperTilt.setSpeed(0);
    return;
  }

  float speed = map(abs(delta), deadzone, 500, 0, currentMaxSpeed);

  // Due to the model having only one tilt limit switch there is secondary one which is virtualy enforced with maxPosition 
  if (digitalRead(tiltLimitSwitch) == LOW && delta < 0) {
    stepperTilt.setSpeed(0);
    return;
  }

  if (delta > 0 && tiltCurentPosition < maxTiltPos) {
    stepperTilt.setSpeed(speed);               // Positive - up
  } else if (delta < 0 && tiltCurentPosition > 0) {
    stepperTilt.setSpeed(-speed);              // Negative - down
  } else {
    stepperTilt.setSpeed(0);
  }

  stepperTilt.runSpeed();
}
