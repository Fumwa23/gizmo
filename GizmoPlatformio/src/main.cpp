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
      Serial.print("Pulse count : ");
      Serial.println(pulseCount);
      pulseCount--;
    }

    lastTime = currentTime;
  }

  if (lastPulseCount != pulseCount){
    const float resonantTimePeriod = 700;

    newOscillationDirection = 0; 
    newOscillationAmplitude = pulseCount*20/maxPulseCount; // This mean that at max pulse count, the amplitude will be at 30
    newTimePeriod = 4*resonantTimePeriod - 3*pulseCount*resonantTimePeriod/maxPulseCount; // This means that at max pulse count, the time period will be at resonant frequency

    newTimePeriodBool = true;
    newOscillationAmplitudeBool = true;
    newOscillationDirectionBool = true;
  }

  // TODO: May want there to be a wider margin range where the pulse count is correct. 
  

  //circularOscillation();
  dynamicOscillation();
  //doOscillation();

  //testingFunction(90);
  //manualCircularOscillation();
  //circularOscillationOwen();
}
