#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include <Arduino.h>
#include <driver/ledc.h>
#include <math.h>
#include <PIDController.h>
#include <SPMController.h>

// --------------------------------------------- CREATE OBJECTS
extern PIDController pid1;
extern PIDController pid2;
extern SPMController spm;

// --------------------------------------------- DEFINE PINS
#define C1_PIN 35
#define C2_PIN 34
#define M1_PIN 12
#define M2_PIN 13

#define C3_PIN 32
#define C4_PIN 33
#define M3_PIN 26
#define M4_PIN 25

#define PULSE_PIN 27
#define REST_PIN 14

// --------------------------------------------- FUNCTION DECLARATIONS
void IRAM_ATTR handleEncoder();
void IRAM_ATTR handleEncoder2();
void moveArmsToHome();
float moveMotorAtSpeed();
float moveTo();
void analogWrite(int motorNumber, float inputPWM, bool remap = true);
void startOscillation(int direction, int magnitude);
void doOscillation();

void setupFunction();

// --------------------------------------------- DEFINE CONSTANTS
const int freq = 30000;
const int resolution = 8;

const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int pwmChannel3 = 2;
const int pwmChannel4 = 3;

const int ENCODER_PULSES_PER_REV = 700;
const float GYZ = ENCODER_PULSES_PER_REV * 3.5 / 360.0;

<<<<<<< HEAD
const float kp = 0.4; // Proportional gain
const float ki = 0.05; // Integral gain
const float kd = 0.2; // Derivative gain
=======
const float kp = 0.4;
const float ki = 0.0; //0.05
const float kd = 0.0; //0.2
>>>>>>> origin/Oscillation-algorithm

const float outMin = -155.0;
const float outMax = 155.0;
const float sampleTime = 0.0001;
const float tau = 0.0001;

const double pi = 3.141592653589793;
const float timePeriod = 2000 * pi * sqrt(0.06 / 9.8);

// --------------------------------------------- DEFINE GLOBAL VARIABLES
extern volatile int encoder1Position;
extern volatile int encoder2Position;

extern unsigned long lastTime;
extern unsigned long lastTime2;

extern float motorAngle1;
extern float motorAngle2;
<<<<<<< HEAD
=======

//Varibales for Oscillation
extern int dOscillationDirection;
extern int mOscillationMagnitude;
extern unsigned long sOscillationStart;
>>>>>>> origin/Oscillation-algorithm

extern float setpoint;

#endif
