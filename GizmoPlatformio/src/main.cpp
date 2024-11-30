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

unsigned long lastCircularOscillationTime = 0;
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

int dOscillationDirection = 0;
int aOscillationAmplitude = 30;

unsigned long sOscillationStart;
unsigned long lastOscillationTime;

float timePeriod = 700;

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
      Serial.print("Pulse count : ");
      Serial.println(pulseCount);
      pulseCount--;
    }

    lastTime = currentTime;
  }

  // aOscillationAmplitude = pulseCount*20/maxPulseCount; // This mean that at max pulse count, the amplitude will be at 30

  // // TODO: May want there to be a wider margin range where the pulse count is correct. 
  // timePeriod = 4*resonantTimePeriod - 3*pulseCount*resonantTimePeriod/maxPulseCount; // This means that at max pulse count, the time period will be at resonant frequency

  circularOscillation();
  // //dynamicOscillation(90, aOscillationAmplitude);
}
