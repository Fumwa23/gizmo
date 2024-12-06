#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include <Arduino.h>
#include <driver/ledc.h>
#include <math.h>
#include <PIDController.h>
#include <SPMController.h>
#include <SPMControllerOwen.h>

// --------------------------------------------- CREATE OBJECTS
extern PIDController pid1;
extern PIDController pid2;
extern SPMController spm;
extern SPMControllerOwen spmOwen;

// --------------------------------------------- DEFINE PINS
#define C1_PIN 35
#define C2_PIN 34
#define M1_PIN 12
#define M2_PIN 13

#define C3_PIN 32
#define C4_PIN 33
#define M3_PIN 26
#define M4_PIN 25

#define PULSE_PIN 14
#define REST_PIN 27

// --------------------------------------------- DEFINE CONSTANTS

// PWM constants
const int freq = 30000;
const int resolution = 8;

// Two PWM channels per motor
const int pwmChannel1 = 0; 
const int pwmChannel2 = 1;
const int pwmChannel3 = 2;
const int pwmChannel4 = 3;

// PID Controller constants
const float kp = 2; // Proportional gain
const float ki = 0.05; // Integral gain
const float kd = 0.0001; // Derivative gain

const float outMin = -200.0;
const float outMax = 200.0; 
const float sampleTime = 0.0001; // Sampling time in seconds
const float tau = 0.0001; // Derivative low-pass filter time constant

// Other constants
const double pi = 3.141592653589793;
const int ENCODER_PULSES_PER_REV = 700;
const float GYZ = ENCODER_PULSES_PER_REV * 3.5 / 360.0; // Ratio of encoder pulses to degrees


// --------------------------------------------- DEFINE GLOBAL VARIABLES
extern volatile int encoder1Position;
extern volatile int encoder2Position;

extern float motorAngle1;
extern float motorAngle2;

// Variables for dial
extern bool lastPulseState;
extern bool dialling;
extern int pulseCount;

// Variables for Oscillation
extern int dOscillationDirection;
extern float aOscillationAmplitude;

extern unsigned long sOscillationStart;
extern unsigned long lastOscillationTime;

extern float timePeriod;

// Timers for asynchronous delay
extern unsigned long lastCircularOscillationTime;
extern unsigned long lastTime;

// --------------------------------------------- FUNCTION DECLARATIONS
void IRAM_ATTR handleEncoder1();
void IRAM_ATTR handleEncoder2();

void moveArmsToHome();
void analogWrite(int motorNumber, float inputPWM, bool remap = true);

void dynamicOscillation();
void circularOscillation();

void setupPins();
void setupMotors();

void trackDialPulses();
void trackNumberDialed();

#endif // PROJECT_CONFIG_H
