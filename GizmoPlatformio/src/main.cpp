/*
This is the main file for the Gizmo Platformio project.

This file contains the setup and loop functions.

It is the file that is compiled and uploaded to the ESP32.
*/

#include <Arduino.h>
#include <driver/ledc.h>
#include <PIDController.h>
#include <SPMController.h>

// --------------------------------------------- CREATE OBJECTS
PIDController pidB;
PIDController pidA;
SPMController spm;

// --------------------------------------------- DEFINE PINS 
//  Motor 1
#define C1_PIN 35 // Encoder channel C1
#define C2_PIN 34 // Encoder channel C2
#define M1_PIN 12 // H-bridge control pin 1
#define M2_PIN 13 // H-bridge control pin 2

// Motor 2
#define C3_PIN 32 // Encoder channel C1
#define C4_PIN 33 // Encoder channel C2
#define M3_PIN 26 // H-bridge control pin 1
#define M4_PIN 25 // H-bridge control pin 2

// Dial
#define PULSE_PIN 14 //Counts clicks of the phone as it unspools
#define REST_PIN 15 //If the dial is in rest position



// --------------------------------------------- FUNCTION DECLARATIONS
void IRAM_ATTR handleEncoder();
void IRAM_ATTR handleEncoder2();
void moveArmsToHome();
float moveMoterAtSpeed();
float moveTo();

void analogWrite(int motorNumber, float inputPWM, bool remap = true); // Custom analogWrite() function

void startOscillation(int magnitude, int direction);
void doOscillation();
// --------------------------------------------- DEFINE CONSTANTS 
// Constants for PWM
const int freq = 30000;
const int resolution = 8;

// PWM channels for motor 1/b
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;

// PWM channels for motor 2/a
const int pwmChannel3 = 2;
const int pwmChannel4 = 3;

// Constants for the encoders
const int ENCODER_PULSES_PER_REV = 700; // Replace with your encoder's PPR

// Constant multiplier to convert degrees to encoder counts
// We have named this like a unit, so 120 degrees is 120 * GYZ
const float GYZ = ENCODER_PULSES_PER_REV * 3.5 / 360.0;

// Constants for PID control
const float kp = 0.4;
const float ki = 0.05;
const float kd = 0.2;

const float outMin = -255.0;
const float outMax = 255.0;
const float sampleTime = 0.0001;
const float tau = 0.0001;

//Define mathematical and sytem constants
const double pi = 2*acos(0);
const float timePeriod = 2*pi*sqrt(0.06/9.8);


// Define collision tolerance angle
const double motorSize = 10.81*GYZ;


// --------------------------------------------- DEFINE GLOBAL VARIABLES 
// Variables for encoder
volatile int bEncoderPosition = 0;
volatile int aEncoderPosition = 0;

// Variables for millis()
unsigned long lastTime = 0;
unsigned long lastTime2 = 0;

//Motor angle  variables. If there are already variables, remove these and add pre-existing variables to the definition later
double aMotorAngle = 240;
double bMotorAngle = 120;

//Variables for dial
bool pulsed = LOW;
bool dialling = false;
int pulseCount = 0;
bool oscillating = false;

//Varibales for Oscillation
int dOscillationDirection;
int mOscillationMagnitude;
unsigned long soscillationStart;

float setpoint = 0; // FOR TESTING ONLY

void setup() {

  // --------------------------------------------- PIN SETUP
  pinMode(C1_PIN, INPUT_PULLUP);
  pinMode(C2_PIN, INPUT_PULLUP);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  pinMode(C3_PIN, INPUT_PULLUP);
  pinMode(C4_PIN, INPUT_PULLUP);
  pinMode(M3_PIN, OUTPUT);
  pinMode(M4_PIN, OUTPUT);

  pinMode(PULSE_PIN, INPUT_PULLUP);
  pinMode(REST_PIN, INPUT_PULLUP);


  // --------------------------------------------- SETUP FUNCTIONS
  // Setup Serial Monitor
  Serial.begin(115200);

  // Attach interrupts to their corresponding functions
  attachInterrupt(digitalPinToInterrupt(C1_PIN), handleEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(C3_PIN), handleEncoder2, RISING);

  // Setup PWM channels for motor 1
  ledcAttachPin(M1_PIN, pwmChannel1);
  ledcSetup(pwmChannel1, freq, resolution);

  ledcAttachPin(M2_PIN, pwmChannel2); 
  ledcSetup(pwmChannel2, freq, resolution);

  // Setup PWM channels for motor 2
  ledcAttachPin(M3_PIN, pwmChannel3); // Attach PWM to channel 0
  ledcSetup(pwmChannel3, freq, resolution);

  ledcAttachPin(M4_PIN, pwmChannel4); // Attach PWM to channel 0
  ledcSetup(pwmChannel4, freq, resolution);

  // Setting up PID 
  pidB.initialise(kp, ki, kd, outMin, outMax, sampleTime, tau);
  pidA.initialise(kp, ki, kd, outMin, outMax, sampleTime, tau);

  //Setting up SPM
  spm.begin(&aMotorAngle, &bMotorAngle);

  // --------------------------------------------- CALLIBRATION AND HOMING

  lastTime = millis();
 
  // Reset encoders positions
  bEncoderPosition = motorSize;
  aEncoderPosition = 360*GYZ - motorSize;

  // Move arms to home position
  moveArmsToHome();




  setpoint = 360 * GYZ; // 120 degrees
  //Ben - Surely this sets it to 360
}

void loop() {

  // Get current time
  unsigned long currentTime = millis();

  // Testing PID
  float bCalculatedPWM = pidB.move(setpoint, bEncoderPosition); // Get PID output (value between -255 AND 255)
  //analogWrite(1, bCalculatedPWM);

  float aCalculatedPWM = pidA.move(5000, aEncoderPosition); // Get PID output (value between -255 AND 255)
  //analogWrite(2, aCalculatedPWM);

  if (currentTime - lastTime >= 1000){

    lastTime = currentTime;

    Serial.print("Setpoint: ");
    Serial.print(setpoint);

    // Print output
    Serial.print(" | Output: ");
    Serial.print(bCalculatedPWM);

    Serial.print(" bEncoderPosition: ");
    Serial.print(bEncoderPosition);

    // Print output
    Serial.print(" | Output2: ");
    Serial.print(aCalculatedPWM);

    Serial.print(" aEncoderPosition: ");
    Serial.println(aEncoderPosition);
  }

  //Code to read phone dial
  //Read rest pin
  bool restState = digitalRead(REST_PIN);

  if (restState){
    //Dial is not in rest state.
    if (!dialling){
      //Just started dialling
      dialling = true;
    }
    //Read pulse pin
    bool pulseState = digitalRead(PULSE_PIN);
    if (pulseState && !pulsed){
      pulseCount++;
    }
    //set pulsed to hold previous value for edge detection
    pulsed = pulseState;
  }else{
    //Dial is in rest state. Check if it has just returned
    if (dialling){
      //Just finished dialling
      dialling = false;
      //DO SOMETHING
      pulseCount = 0;
    }
  }

  if (oscillating){
    doOscillation();
  }else{
    startOscillation(10,0);
  }

  // if (currentTime - lastTime2 >= 2500){

  //   lastTime2 = currentTime;
    

  //   if (setpoint == 2400) {
  //     setpoint = 0;
  //   }
  //   else {
  //     setpoint = 2400;
  //   }
  // }
}



// --------------------------------------------- FUNCTIONS DEFINITIONS

// Interrupt function 1
void IRAM_ATTR handleEncoder() {
    int state1 = digitalRead(C1_PIN);
    int state2 = digitalRead(C2_PIN);

    if (state1 != state2) {
        // Clockwise
        bEncoderPosition++;
    } else {
        // Anti-clockwise
        bEncoderPosition--;
    }
}

// Interrupt function 2
void IRAM_ATTR handleEncoder2() {
    int state1 = digitalRead(C3_PIN);
    int state2 = digitalRead(C4_PIN);

    if (state1 != state2) {
        // Clockwise
        aEncoderPosition++;
    } else {
        // Anti-clockwise
        aEncoderPosition--;
    }
}

/**
 * @brief Sends a PWM signal to control a motor's speed and direction.
 * 
 * This function writes a PWM signal to a specified motor. It can optionally remap
 * the input PWM value to ensure the motor operates within a valid range.
 * The motor's direction (clockwise or anti-clockwise) is determined by the sign 
 * of the `inputPWM` value.
 * 
 * @param motorNumber The motor to control (1 for motor 1, 2 for motor 2).
 * @param inputPWM The PWM signal to send to the motor. Positive values indicate
 *                 clockwise rotation, and negative values indicate anti-clockwise 
 *                 rotation.
 * @param remap If true (default), adjusts the PWM value to ensure the motor operates within 
 *              its minimum and maximum range. 
 */
void analogWrite(int motorNumber, float inputPWM, bool remap){

  // The motor requires a minimum pwm to move, this remaps ths values
  if (remap){ 
    if (inputPWM < 0) {
      inputPWM -= 255 + outMin;
    } else if (inputPWM > 0) {
      inputPWM += 255 - outMax;
    }
  }

  if (motorNumber == 1){
    // Write output to motor 1
    if (inputPWM > 0) {
      // Clockwise
      ledcWrite(pwmChannel1, 255 - inputPWM);
      ledcWrite(pwmChannel2, 255);
    } else {
      // Anti-clockwise
      ledcWrite(pwmChannel1, 255);
      ledcWrite(pwmChannel2, 255 - inputPWM * -1);
    }
  }

  if (motorNumber == 2){
    // Write output to motor 2
    if (inputPWM > 0) {
      // Clockwise
      ledcWrite(pwmChannel3, 255 - inputPWM);
      ledcWrite(pwmChannel4, 255);
    } else {
      // Anti-clockwise
      ledcWrite(pwmChannel3, 255);
      ledcWrite(pwmChannel4, 255 - inputPWM * -1);
    }
  }
}

/**
 * @brief Moves the robot's arms to their home positions.
 * 
 * Uses PID control to move Motor 1 to 120 degrees and Motor 2 to 240 degrees.
 * The function continuously adjusts PWM signals and checks encoder feedback 
 * until both motors reach their targets within a tolerance of ±14.
 * 
 * Outputs "---- HOMING COMPLETE ----" to the serial monitor when done.
 * 
 * @note This is a blocking function that pauses program execution until 
 *       the target positions are reached.
 */
void moveArmsToHome() {
  unsigned long startTime = millis(); // Start time for timeout
  const unsigned long timeout = 1000;

  while (true){
    float bCalculatedPWM = pidB.move(120*GYZ, bEncoderPosition); 
    analogWrite(1, bCalculatedPWM);

    // Move motor 2 to 240 degrees
    float aCalculatedPWM = pidA.move(240*GYZ, aEncoderPosition);
    analogWrite(2, aCalculatedPWM);

    // Check to see if position has been reached
    if (abs(bEncoderPosition - 120*GYZ) < 50 && abs(aEncoderPosition - 240*GYZ) < 50){
      Serial.println("---- HOMING COMPLETE ----");
      //delay(10000);
      break;
    }

    if (millis() - startTime > timeout) {
      Serial.print("---- WAITING FOR HOME TO COMPLETE ----");
      
      // Print output
      Serial.print(" | Output: ");
      Serial.print(bCalculatedPWM);

      Serial.print(" bEncoderPosition: ");
      Serial.print(bEncoderPosition);

      // Print output
      Serial.print(" | Output2: ");
      Serial.print(aCalculatedPWM);

      Serial.print(" aEncoderPosition: ");
      Serial.println(aEncoderPosition);

      startTime = millis();
    }
  }
}

float moveMotorAtSpeed(){
  // code here
  return 0;
}


//Oscillation functions
void startOscillation(int direction, int magnitude){
  dOscillationDirection = direction;
  mOscillationMagnitude = magnitude;
  oscillating = true;
  spm.calculate_motors(mOscillationMagnitude,dOscillationDirection);
  float bTarget = bMotorAngle*GYZ;
  float aTarget = aMotorAngle*GYZ;
  

}