/*
This is the main file for the Gizmo Platformio project.

This file contains the setup and loop functions.

It is the file that is compiled and uploaded to the ESP32.
*/

#include <Arduino.h>
#include <driver/ledc.h> // Library replacement for analogWrite - Allows frequency, channel and resolution control.
#include <PIDController.h>

PIDController pid;

// Define pins
#define C1_PIN 26 // Encoder channel C1
#define C2_PIN 27 // Encoder channel C2
#define M1_PIN 12 // H-bridge control pin 1
#define M2_PIN 13 // H-bridge control pin 2

// put function declarations here:
void home();


// Constants for PWM
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

// Variables for encoder
volatile int encoderPosition = 0;
unsigned long lastTime = 0;

// Constants for encoder
const int ENCODER_PULSES_PER_REV = 700; // Replace with your encoder's PPR

// Constant multiplier to convert degrees to encoder counts
// We have named this like a unit, so 120 degrees is 120 * GYZ
const float GYZ = ENCODER_PULSES_PER_REV * 3.5 / 360.0;

// Interrupt service routine for encoder
void IRAM_ATTR handleEncoder() {
  if (digitalRead(C2_PIN) == LOW) {
    encoderPosition--;
  } else {
    encoderPosition++;
  }
}

// PID Setup
const float kp = 0.5;
const float ki = 0.0;
const float kd = 0.0;
const float outMin = -255.0;
const float outMax = 255.0;
const float sampleTime = 0.001;
const float tau = 0.0001;

void setup() {
  Serial.begin(115200);

  // Setup pins as Input/Output
  pinMode(C1_PIN, INPUT_PULLUP);
  pinMode(C2_PIN, INPUT_PULLUP);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(C1_PIN), handleEncoder, RISING);

  ledcAttachPin(M1_PIN, pwmChannel); // Attach PWM to channel 0
  ledcSetup(pwmChannel, freq, resolution);

  lastTime = millis();
 
  home(); // Call home function to calibrate the arms

  // Setting up PID controller
  pid.initialise(kp, ki, kd, outMin, outMax, sampleTime, tau);
}

void loop() {
    // Testing PID
    float setpoint = 700.0;
    float measurement = encoderPosition;

    float output = pid.move(setpoint, measurement);


  //ledcWrite(pwmChannel, pwmValue); // Write PWM value to channel 0
  ledcWrite(pwmChannel, 255); // Write PWM value to channel 0

  // Ensure M2 is LOW for unidirectional control
  digitalWrite(M2_PIN, LOW);

  Serial.print("EncoderPosition: ");
  Serial.print(encoderPosition);

  // Print output
  Serial.print(" | Output: ");
  Serial.println(output);

  // Print encoderPosition every second
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 1000) { // 1-second interval
    lastTime = currentTime;
    
  }
}


// put function definitions here:
void home() {
  // Move one arm 120 degrees clockwise
    // use relative PID to move 120 clockwise

  // Move the other arm 120 degrees anticlockwise
    // use relative PID to move 120 anticlockwise

  // Set EncoderHome position to 0 for both arms
  
}