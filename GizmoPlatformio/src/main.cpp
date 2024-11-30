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

// --------------------------------------------- DEFINE GLOBAL VARIABLES 
// Variables for encoder
volatile int encoder1Position = 0;
volatile int encoder2Position = 0;

unsigned long lastTime = 0;
unsigned long lastTime2 = 0;

//Motor angle variables. If there are already variables, remove these and add pre-existing variables to the definition later
float motorAngle1 = 120;
float motorAngle2 = 240;

//Variables for dial
bool pulsed = LOW;
bool dialling = false;
int pulseCount = 0;

//Varibales for Oscillation
bool oscillating = false;
int dOscillationDirection;
int mOscillationAmplitude;
unsigned long sOscillationStart;
unsigned long lastOscillationTime;

//Variables for speed testing
bool testing = false;
int direction = 1;
unsigned long startTestTime = 0;
unsigned long lastTestSample = 0;

float setpoint = 0; // FOR TESTING ONLY

void setup() {
  // Setup Serial Monitor
  Serial.begin(115200);

  setupPins();
  setupMotors();


  Serial.print("Time period : ");
  Serial.println(timePeriod);

  // Move arms to home position
  moveArmsToHome();
  delay(1000);
}

void loop() {

  // Get current time
  unsigned long currentTime = millis();

  //Code to read phone dial
  //Read rest pin
  bool restState = digitalRead(REST_PIN);

  if (restState){
    //Dial is not in rest state.
    if (!dialling){
      //Just started dialling
      dialling = true;
    }
    //Read pulse pin
    bool pulseState = digitalRead(PULSE_PIN);
    if (pulseState && !pulsed){
      pulseCount++;
    }
    //set pulsed to hold previous value for edge detection
    pulsed = pulseState;
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
  testSpeed(30);
  // if (oscillating){
  //   doOscillation();
  // }else{
  //   startOscillation(0,42.75);
  //   delay(1000);
  // }
}
