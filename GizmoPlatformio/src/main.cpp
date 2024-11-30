/*
This is the main file for the Gizmo Platformio project.

This file contains the setup and loop functions.

It is the file that is compiled and uploaded to the ESP32.
*/

#include "projectConfig.h"

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

//Motor angle variables. If there are already variables, remove these and add pre-existing variables to the definition later
float motorAngle1 = 120;
float motorAngle2 = 240;

//Variables for dial
bool pulsed = LOW;
bool dialling = false;
int pulseCount = 0;
bool oscillating = false;

//Varibales for Oscillation
int dOscillationDirection;
int mOscillationMagnitude;
unsigned long sOscillationStart;
unsigned long lastOscillationTime;

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
  pid1.initialise(kp, ki, kd, outMin, outMax, sampleTime, tau);
  pid2.initialise(kp, ki, kd, outMin, outMax, sampleTime, tau);
  spm.begin(&motorAngle2, &motorAngle1);

  // --------------------------------------------- CALLIBRATION AND HOMING

  lastTime = millis();
 
  // Starting encoder positions on setup.
  encoder1Position = 33.5*GYZ;
  encoder2Position = (360-33.5)*GYZ;


  Serial.print("Time period : ");
  Serial.println(timePeriod);

  // Move arms to home position
  moveArmsToHome();
  delay(1000);
}

void loop() {

  // Get current time
  unsigned long currentTime = millis();

  // Tracking position every second for debugging
  if (currentTime - lastTime >= 1000){

    lastTime = currentTime;

    // Print output
    Serial.print(" | Output: ");
    Serial.print("INSERT PWM OUTPUT");

    Serial.print(" Encoder1Position: ");
    Serial.print(encoder1Position);

    // Print output
    Serial.print(" | Output2: ");
    Serial.print("INSERT PWM OUTPUT");

    Serial.print(" Encoder2Position: ");
    Serial.print(encoder2Position);
    Serial.println(" |");
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
    startOscillation(0,30);
    delay(1000);
  }
}



// --------------------------------------------- FUNCTIONS DEFINITIONS

// Interrupt function 1
void IRAM_ATTR handleEncoder() {
    int state1 = digitalRead(C1_PIN);
    int state2 = digitalRead(C2_PIN);

    if (state1 != state2) {
        // Clockwise
        encoder1Position++;
    } else {
        // Anti-clockwise
        encoder1Position--;
    }
}

// Interrupt function 2
void IRAM_ATTR handleEncoder2() {
    int state1 = digitalRead(C3_PIN);
    int state2 = digitalRead(C4_PIN);

    if (state1 != state2) {
        // Clockwise
        encoder2Position++;
    } else {
        // Anti-clockwise
        encoder2Position--;
    }
}


float moveMotorAtSpeed(){
  // code here
  return 0;
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
 * @todo Change map function to be proper mapping
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
    float calculatedPWM1 = pid1.move(120*GYZ, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    // Move motor 2 to 240 degrees
    float calculatedPWM2 = pid2.move(240*GYZ, encoder2Position);
    analogWrite(2, calculatedPWM2);

    // Check to see if position has been reached
    if (abs(encoder1Position - 120*GYZ) < 50 && abs(encoder2Position - 240*GYZ) < 50){
      Serial.println("---- HOMING COMPLETE ----");
      analogWrite(1, 0.0); // Stop motors 
      analogWrite(2, 0.0); // Stop motors
      break;
    }

    if (millis() - startTime > timeout) {
      Serial.print("---- WAITING FOR HOME TO COMPLETE ----");
      
      // Print output
      Serial.print(" | Output: ");
      Serial.print(calculatedPWM1);

      Serial.print(" Encoder1Position: ");
      Serial.print(encoder1Position);

      // Print output
      Serial.print(" | Output2: ");
      Serial.print(calculatedPWM2);

      Serial.print(" aEncoderPosition: ");
      Serial.println(encoder2Position);

      startTime = millis();
    }
  }
}


//Oscillation functions
void startOscillation(int direction, int magnitude){
  Serial.print(millis());
  Serial.println(" ---- STARTING OSCILATTION ---- ");
  dOscillationDirection = direction;
  mOscillationMagnitude = magnitude;
  sOscillationStart = millis();
  oscillating = true;
  Serial.println(magnitude);
  spm.calculate_motors(mOscillationMagnitude,dOscillationDirection);
  float target1 = motorAngle1*GYZ;
  float target2 = motorAngle2*GYZ;
  Serial.print("Target 1 : ");
  Serial.print(target1);
  Serial.print("   Target 2 : ");
  Serial.println(target2);
  unsigned long startTime = millis(); // Start time for timeout
  const unsigned long timeout = 100;
  while (true){
    //Move motors to targets
    float calculatedPWM1 = pid1.move(target1, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    float calculatedPWM2 = pid2.move(target2, encoder2Position);
    analogWrite(2, calculatedPWM2);

    // Check to see if position has been reached
    if (abs(encoder1Position - target1) < 50 && abs(encoder2Position - target2) < 50){
      Serial.print(millis());
      Serial.println("---- OSCILLATION START ----");
      lastOscillationTime = millis();

      analogWrite(1, 0.0);
      analogWrite(2, 0.0);
      break;
    }
    if (millis() - startTime > timeout) {
      Serial.print("---- WAITING TO START OSCILLATION ----");
      
      // Print output
      Serial.print(" | Output: ");
      Serial.print(calculatedPWM1);

      Serial.print(" Encoder1Position: ");
      Serial.print(encoder1Position);

      // Print output
      Serial.print(" | Output2: ");
      Serial.print(calculatedPWM2);

      Serial.print(" Encoder2Position: ");
      Serial.println(encoder2Position);

      startTime = millis();
    }
  }
}

void doOscillation(){
  if (millis() > lastOscillationTime+20){
    float t = fmod(millis()-sOscillationStart,timePeriod);
    float phi = mOscillationMagnitude*cos(2*pi*t/timePeriod);
    spm.calculate_motors(phi, dOscillationDirection);
    float calculatedPWM1 = pid1.move(motorAngle1*GYZ, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    float calculatedPWM2 = pid2.move(motorAngle2*GYZ, encoder2Position);
    analogWrite(2, calculatedPWM2);
    Serial.print("t : ");
    Serial.print(t);
    Serial.print("   Phi : ");
    Serial.print(phi);
    Serial.print("   PWM1 : ");
    Serial.print(calculatedPWM1);
    Serial.print("   Encoder1 : ");
    Serial.print(encoder1Position/GYZ);
    Serial.print("   Target1  : ");
    Serial.print(motorAngle1);
    Serial.print("   PWM2 : ");
    Serial.print(calculatedPWM2);
    Serial.print("   Encoder2 : ");
    Serial.print(encoder2Position/GYZ);
    Serial.print("   Target2 : ");
    Serial.println(motorAngle2);
    lastOscillationTime = millis();
  }
}