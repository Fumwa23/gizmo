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

float timePeriod = 0;

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
int aOscillationAmplitude = 20;
int newOscillationDirection = 0;
int newOscillationAmplitude = 0;
bool newOscillationDirectionBool = false;
bool newOscillationAmplitudeBool = false;

unsigned long sOscillationStart;
unsigned long lastOscillationTime;

float timePeriod = 650;
float newTimePeriod = 0;
bool newTimePeriodBool = false;

const int thetaTimePeriod = 1000;
const int phiTimePeriod = 1000;
const int phiMaxTimePeriod = 5000;
const int phiMinTimePeriod = 100;


// Manual circular Oscillation variable
int stage = 0;

void setup() {
  Serial.begin(115200);

  setupPins();
  setupMotors();

  moveArmsToHome();
  delay(1000);

  sOscillationStart = millis(); // TODO: check if this goes here.
}

void loop() {

  // Get current time
  getTime();

  trackDialPulses();


  // Track pulses
  trackPulses();
  //timePeriod = 50 * pulseCount;

  if (currentTime - lastTime >= 1000) {
    // Calculate RPM every second
    pulseCount -= 1;
    
    if (pulseCount < 0){
      pulseCount = 0;
    } 

    Serial.print("Pulse Count: ");
    Serial.println(pulseCount);
  }

  //circularOscillation();
  dynamicOscillation(dOscillationDirection, aOscillationAmplitude);
  // if (oscillating){
  //   doOscillation();
  // }else{
  //   startOscillation(0,42.75);
  //   delay(1000);
  // }
}
