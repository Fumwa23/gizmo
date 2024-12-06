/**
 * @file main.cpp
 * @brief Main file for the robot arm project.
 * 
 * This file contains the setup and loop functions for the robot arm project.
 * 
 * @note This file is the entry point for the program.
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

unsigned long lastTime = 0;
unsigned long lastCircularOscillationTime = 0;

//Motor angle variables. If there are already variables, remove these and add pre-existing variables to the definition later
float motorAngle1 = 120;
float motorAngle2 = 240;

//Variables for dial
bool lastPulseState = LOW;
bool dialling = false;
int pulseCount = 0;

//Varibales for Oscillation

int dOscillationDirection = 0;
float aOscillationAmplitude = 0;

unsigned long sOscillationStart;
unsigned long lastOscillationTime;

float timePeriod = 650;

void setup() {
  Serial.begin(115200);

  setupPins();
  setupMotors();

  moveArmsToHome();
  delay(1000);

  sOscillationStart = millis(); // TODO: check if this goes here.
}

void loop() {
  unsigned long currentTime = millis();

  trackDialPulses(); // get number of pulses

  // every 100ms be subtracting a bit from amplitude
  const float resonantTimePeriod = 700;
  timePeriod = resonantTimePeriod;
  const float maxPulseCount = 300;
  const float maxAmplitude = 30;

  if (pulseCount > maxPulseCount){
    pulseCount = maxPulseCount;
  }

  const int stepsTillMax = 100;

  if (currentTime - lastTime > 100){

    if (aOscillationAmplitude > 0){
      aOscillationAmplitude -= (maxAmplitude/stepsTillMax);
    }

    // every 100ms be taking a bit from the pulse count and adding it to amplitude and time period
    if (pulseCount > 0){
      if (aOscillationAmplitude < maxAmplitude){
        aOscillationAmplitude += (maxAmplitude/stepsTillMax)*3;
      }
      
      pulseCount -= 3;
    }

    dOscillationDirection = (dOscillationDirection + 2) % 360;

    Serial.print("Pulse count: ");
    Serial.print(pulseCount);
    Serial.print(" | Amplitude: ");
    Serial.print(aOscillationAmplitude);
    Serial.print(" | Time period: ");
    Serial.println(timePeriod);

    lastTime = currentTime;
  }

  dynamicOscillation();
}
