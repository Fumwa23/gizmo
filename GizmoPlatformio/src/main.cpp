/*
This is the main file for the Gizmo Platformio project.

This file contains the setup and loop functions.

It is the file that is compiled and uploaded to the ESP32.
*/

#include <Arduino.h>
#include <driver/ledc.h> // Library replacement for analogWrite - Allows frequency, channel and resolution control.

// put function declarations here:
void home();

// Define pins
#define C1_PIN 26 // Encoder channel C1
#define C2_PIN 27 // Encoder channel C2
#define MOTOR_PWM_PIN1 12 // H-bridge control pin 1
#define MOTOR_PWM_PIN2 13 // H-bridge control pin 2

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

// Variables for encoder
volatile int encoderCount = 0;
volatile int encoderCount2 = 0;
unsigned long lastTime = 0;

// Constants for encoder
const int ENCODER_PULSES_PER_REV = 700; // Replace with your encoder's PPR

// Constant multiplier to convert degrees to encoder counts
// We have named this like a unit, so 120 degrees is 120 * GYZ
const float GYZ = ENCODER_PULSES_PER_REV * 3.5 / 360.0;

// Interrupt service routine for encoder
void IRAM_ATTR handleEncoder() {
  encoderCount++;
}
void IRAM_ATTR handleEncoder2() {
  encoderCount2++;
}

void setup() {
  Serial.begin(115200);
  pinMode(C1_PIN, INPUT_PULLUP);
  pinMode(C2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(C1_PIN), handleEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(C2_PIN), handleEncoder2, RISING);

  // Setup motor control pins as outputs
  pinMode(MOTOR_PWM_PIN1, OUTPUT);
  pinMode(MOTOR_PWM_PIN2, OUTPUT);

  ledcAttachPin(MOTOR_PWM_PIN1, pwmChannel); // Attach PWM to channel 0
  ledcSetup(pwmChannel, freq, resolution);

  lastTime = millis();
 
  home(); // Call home function to calibrate the arms
}

void loop() {
  //ledcWrite(pwmChannel, pwmValue); // Write PWM value to channel 0
  ledcWrite(pwmChannel, 255); // Write PWM value to channel 0

  // Ensure MOTOR_PWM_PIN2 is LOW for unidirectional control
  digitalWrite(MOTOR_PWM_PIN2, LOW);

  // Calculate RPM every second
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 1000) { // 1-second interval
    float rpm = (encoderCount / (float)ENCODER_PULSES_PER_REV) * 60.0; // Calculate RPM
    lastTime = currentTime;

    Serial.print("EncoderCount: ");
    Serial.print(encoderCount);

    Serial.print(" | Encoder 2 Count: ");
    Serial.print(encoderCount2);
    Serial.print(" | RPM: ");
    Serial.println(rpm);

    encoderCount = 0;
    encoderCount2 = 0;
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