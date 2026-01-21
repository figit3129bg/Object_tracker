// the controller for this sketch is an ESP32 S3 WROOM 1 N16R8

// all of the pins bellow are pins that have ADC and are used for stepper control
// MS1, MS2 and MS3 pins are all set to HIGH and its hardwired into the solder board.
// All of them are conected to 5V

#define step1Pin 9      //GPIO 9
#define dir1Pin 10      //GPIO 10
#define step2Pin 11     //GPIO 11
#define dir2Pin 12      //GPIO 12

// Centering pins are 16, 17 and 18. 
#define tiltLimitSwitch 16
#define panLimitSwitchL 17
#define panLimitSwitchR 18

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
