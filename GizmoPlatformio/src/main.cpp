/*
This is the main file for the Gizmo Platformio project.

This file contains the setup and loop functions.

It is the file that is compiled and uploaded to the ESP32.
*/

#include "projectConfig.h" // Include proect header file

// --------------------------------------------- CREATE OBJECTS
PIDController pid1;
PIDController pid2;
SPMController spm;
SPMControllerOwen spmOwen;

// --------------------------------------------- DEFINE GLOBAL VARIABLES 
// Variables for encoder
volatile int encoder1Position = 0;
volatile int encoder2Position = 0;

unsigned long lastCircularOscillationTime = 0;
unsigned long lastTime = 0;
unsigned long lastTime2 = 0;

//Motor angle variables. If there are already variables, remove these and add pre-existing variables to the definition later
float motorAngle1 = 120;
float motorAngle2 = 240;

//Variables for dial
bool lastPulseState = LOW;
bool dialling = false;
int pulseCount = 0;
int lastPulseCount = 0;

//Varibales for Oscillation
bool oscillating = false;

int dOscillationDirection = 0;
int aOscillationAmplitude = 30;
int newOscillationDirection = 0;
int newOscillationAmplitude = 0;
bool newOscillationDirectionBool = false;
bool newOscillationAmplitudeBool = false;

unsigned long sOscillationStart;
unsigned long lastOscillationTime;

//Variables for speed testing
bool testing = false;
int direction = 1;
unsigned long startTestTime = 0;
unsigned long lastTestSample = 0;


float timePeriod = 700;
float newTimePeriod = 0;
bool newTimePeriodBool = false;


// Manual circular Oscillation variable
int stage = 0;

void setup() {
  Serial.begin(115200);

  setupPins();
  setupMotors();

  // int i;
  // for (i=0; i<=360; i+=45){
  //   spm.calculate_motors(10,i);
  //   Serial.print(" SPM Angle : ");
  //   Serial.print(i);
  //   Serial.print("   Motor1 Angle : ");
  //   Serial.print(motorAngle1);
  //   Serial.print("   Motor2 Angle : ");
  //   Serial.println(motorAngle2);
  // }
  // while(true){}


  moveArmsToHome();
  delay(2000);


  sOscillationStart = millis(); // TODO: check if this goes here.
}

void loop() {

  // Get current time
  unsigned long currentTime = millis();
  //unsigned long currentTime2 = millis();

  trackDialPulses();

  const int maxPulseCount = 30;

  if (pulseCount > maxPulseCount){
    pulseCount = maxPulseCount;
  }

  // TODO: create a function which iteratively decreases the pulseCount until it reaches 0 at a time interval
  if (currentTime - lastTime >= 2000){
    if (pulseCount > 0){
      lastPulseCount = pulseCount;
      pulseCount--;
    }
    //set pulsed to hold previous value for edge detection
  }else{
    //Dial is in rest state. Check if it has just returned
    if (dialling){
      //Just finished dialling
      dialling = false;
      //TO DO  - SOMETHING
      pulseCount = 0;
    }
  }

  int oscillationDirection = 90;
  int oscillationAmplitude = 30;
  testSpeed(20);
  // if (oscillating){
  //   doOscillation();
  // }else{
  //   startOscillation(0,42.75);
  //   delay(1000);
  // }
}
