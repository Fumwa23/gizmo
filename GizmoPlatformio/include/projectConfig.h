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

// --------------------------------------------- FUNCTION DECLARATIONS
void IRAM_ATTR handleEncoder1();
void IRAM_ATTR handleEncoder2();

void moveArmsToHome();
float moveMotorAtSpeed();
float moveTo();
void analogWrite(int motorNumber, float inputPWM, bool remap = true);

void setupFunction();

//Bens Effiecient? matbe? oscillation algorithm
void getTime();
void checkChanges();
int get_momentum();
void doOscillation();

// --------------------------------------------- DEFINE CONSTANTS
const int freq = 30000;
const int resolution = 8;

const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int pwmChannel3 = 2;
const int pwmChannel4 = 3;

const int ENCODER_PULSES_PER_REV = 700;
const float GYZ = ENCODER_PULSES_PER_REV * 3.5 / 360.0;

const float kp = 2; // Proportional gain
const float ki = 0.05; // Integral gain
const float kd = 0.0001; // Derivative gain

const float outMin = -155.0;
const float outMax = 155.0;
const float sampleTime = 0.0001;
const float tau = 0.0001;

const double pi = 3.141592653589793;
extern float timePeriod; // 1 * 1000 * 2 * pi * sqrt(0.06 / 9.8);
extern bool newTimePeriodBool;
extern float newTimePeriod;

const int thetaTimePeriod = 1000;
const int phiTimePeriod;
const int phiMaxTimePeriod = 5000;
const int phiMinTimePeriod = 100;

const int maxMomentumGain = 1;
const int minMomentumGain = -1;
// --------------------------------------------- DEFINE GLOBAL VARIABLES
extern volatile int encoder1Position;
extern volatile int encoder2Position;

extern unsigned long lastTime;
extern unsigned long lastTime2;

extern float motorAngle1;
extern float motorAngle2;

//Phi clock variables
extern int phiTimePeriod = 0;
extern unsigned int tPhi;

//Theta variable
int current_theta;

//Varibales for Oscillation
extern bool doneCentre = false;
int cachedMomentum;
int frequency;
int amplitude;

//New oscillation

//Variables for dial
extern bool lastPulseState;
extern bool dialling;
extern int pulseCount;
extern int lastPulseCount;

extern int stage;

void setupPins();
void setupMotors();

void analogWrite(int motorNumber, float inputPWM, bool remap);
void moveArmsToHome();
float moveMotorAtSpeed();

void IRAM_ATTR handleEncoder1();
void IRAM_ATTR handleEncoder2();

void trackDialPulses();
void trackNumberDialed();

#endif // PROJECT_CONFIG_H
