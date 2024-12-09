#include "projectConfig.h"

// --------------------------------------------- CREATE OBJECTS
PIDController pid1; // PID contoller for first motor
PIDController pid2; // PID controller for second motor
SPMController spm; // Inverse kinematics controller

// --------------------------------------------- DEFINE GLOBAL VARIABLES 
volatile int encoder1Position = 0;
volatile int encoder2Position = 0;

float motorAngle1 = 120;
float motorAngle2 = 240;

// Variables for dial
bool lastPulseState = LOW;
bool dialling = false;
int pulseCount = 0;

//Varibales for Oscillation
int dOscillationDirection = 0;
float aOscillationAmplitude = 0;

unsigned long sOscillationStart; // Start time of the oscillation in milliseconds
unsigned long lastOscillationTime;

float timePeriod = 650; 

// Timers for asynchronous delay
unsigned long lastTime = 0;
unsigned long lastCircularOscillationTime = 0;

void setup() {
  Serial.begin(115200);

  setupPins();
  setupMotors();

  moveArmsToHome();

  lastTime = millis();
  sOscillationStart = millis();
}

void loop() {
  trackDialPulses();

  updateOscillationParameters();

  dynamicallyOscillate();
  // circularOscillation();
}
