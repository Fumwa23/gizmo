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
unsigned long nextPulseDrop = 0; 

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
float aOscillationAmplitude = 0;
float newOscillationAmplitude = 0;
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

unsigned int tPhi;

bool doneCentre = false;

//TESTING VARIABLES
bool increasingPulseCount = true;

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
  unsigned long currentTime = millis();

/*
  trackDialPulses();
  dropPulseCount();

  //testingUpdateParameter();

  updateParameters();

  dynamicOscillation();

*/

  // NEW METHOD:

  // get number of pulses
  trackDialPulses();

  // have amplitude and time period with their own variables
  //aOscillationAmplitude
  //timePeriod

  // every 100ms be subtracting a bit from amplitude and adding a bit to time period
  const float resonantTimePeriod = 700;
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

    if (timePeriod < 4*resonantTimePeriod){
      timePeriod += (3*resonantTimePeriod/stepsTillMax);
    }

    // every 100ms be taking a bit from the pulse count and adding it to amplitude and time period
    if (pulseCount > 0){
      if (aOscillationAmplitude < maxAmplitude){
        aOscillationAmplitude += (maxAmplitude/stepsTillMax)*3;
      }

      if (timePeriod > resonantTimePeriod){
        timePeriod -= (3*resonantTimePeriod/stepsTillMax)*3;
      }
      
      pulseCount -= 3;
    }

    Serial.print("Pulse count: ");
    Serial.print(pulseCount);
    Serial.print(" | Amplitude: ");
    Serial.print(aOscillationAmplitude);
    Serial.print(" | Time period: ");
    Serial.println(timePeriod);

    lastTime = currentTime;
  }


  // update oscillation
  alternativeOscillation();
}
