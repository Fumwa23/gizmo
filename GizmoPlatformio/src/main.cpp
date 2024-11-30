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

float timePeriod = 0;

//Motor angle variables. If there are already variables, remove these and add pre-existing variables to the definition later
float motorAngle1 = 120;
float motorAngle2 = 240;

//Variables for dial
bool pulsed = LOW;
bool dialling = false;
int pulseCount = 0;

//Varibales for Oscillation
bool oscillating = false;

int dOscillationDirection = 90;
int aOscillationAmplitude = 30;

unsigned long sOscillationStart;
unsigned long lastOscillationTime;

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
  unsigned long currentTime2 = millis();


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
