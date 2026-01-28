// ESP32-S3 WROOM-1 N16R8 Controller for Pan & Tilt test
#include <AccelStepper.h>

#define deadzone 20
float currentMaxSpeed = 8000; // adjustable
// float maxSpeed = 5000; //max speed in steps hardwired

// Virtual inputs from phone (default to neutral)
int virtualPanInput = 500;      // 0–1000 neutral is 500
int virtualTiltInput = 500;     // 0–1000 neutral is 500
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
  Serial.println("Full homing done.");

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
    else if(cmd.startsWith("Homing:")){  //Homing: + 1(tilt)/2(pan)/3(both)
      if(cmd.substring(7).toInt()==1){ tiltHoming();}
      else if(cmd.substring(7).toInt()==2){ panHoming();}
      else if(cmd.substring(7).toInt()==3){ tiltHoming(); panHoming();}
    }
  }

  updatePanMotor();
  updateTiltMotor();
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
  if (delta > 0 && (max_pos == 0 || currentPos < max_pos)) {  
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

  tiltCurrentPosition = stepperTilt.currentPosition();

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

  if (delta > 0 && tiltCurrentPosition < maxTiltPos) {
    stepperTilt.setSpeed(speed);               // Positive - up
  } else if (delta < 0 && tiltCurrentPosition > 0) {
    stepperTilt.setSpeed(-speed);              // Negative - down
  } else {
    stepperTilt.setSpeed(0);
  }

  stepperTilt.runSpeed();
}

void tiltHoming(){
  stepperTilt.setSpeed(-3000);
  while (digitalRead(tiltLimitSwitch) == HIGH) {stepperTilt.runSpeed();}
  stepperTilt.setCurrentPosition(0);
}

void panHoming(){
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
  max_pos = rightSteps;
  Serial.print("Right Steps: ");
  Serial.println(rightSteps);      
      
  // Centering the Z axis
  centerPanPosition = max_pos/2;
  stepperPan.moveTo(centerPanPosition);
  stepperPan.runToPosition();

}
/*
void updateRGBBlink(){
  unsigned long currentMillis = millis();

if (!isHomingTilt && !isHomingPan && !isHomingBoth) {
    // No homing → turn LED off
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    blinkCount = 0;
    ledOn = false;
    return;
  }
  const int* pattern = (isHomingTilt && !isHomingBoth) ? tiltPattern :
                       (isHomingPan  && !isHomingBoth) ? panPattern  :
                       tiltPattern;
  
  if (currentMillis - previousBlinkMillis >= (ledOn ? fastBlinkInterval : pauseAfterSequence)) {
    previousBlinkMillis = currentMillis;
    if (ledOn) {
      // Turn LED OFF
      neopixelWrite(RGB_BUILTIN, 0, 0, 0);
      ledOn = false;
      blinkCount++;
    }
    else {
      // Turn LED ON (red) if we haven't finished the sequence
      if (pattern[blinkCount] > 0) {
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);  // Full red
        ledOn = true;
      } else {
        // Sequence finished → reset and start over
        blinkCount = 0;
      }
    }
  }

}
}
*/
