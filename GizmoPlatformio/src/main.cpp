/*
This is the main file for the Gizmo Platformio project.

This file contains the setup and loop functions.

It is the file that is compiled and uploaded to the ESP32.
*/

#include <Arduino.h>
#include <driver/ledc.h> // Library replacement for analogWrite - Allows frequency, channel and resolution control.
#include <PIDController.h>
#include <SPMController.h>

PIDController pid;
SPMController spm;

// Define pins
#define C1_PIN 26 // Encoder channel C1
#define C2_PIN 27 // Encoder channel C2
#define M1_PIN 12 // H-bridge control pin 1
#define M2_PIN 13 // H-bridge control pin 2

// put function declarations here:
void home();


// Constants for PWM
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 200;

// Variables for encoder
volatile int encoderPosition = 0;
volatile int lastState = 0;

unsigned long lastTime = 0;

// Constants for encoder
const int ENCODER_PULSES_PER_REV = 700; // Replace with your encoder's PPR

// Constant multiplier to convert degrees to encoder counts
// We have named this like a unit, so 120 degrees is 120 * GYZ
const float GYZ = ENCODER_PULSES_PER_REV * 3.5 / 360.0;

void IRAM_ATTR handleEncoder() {
    int state1 = digitalRead(C1_PIN);
    int state2 = digitalRead(C2_PIN);

    if (state1 != state2) {
        // Clockwise
        encoderPosition++;
    } else {
        // Anti-clockwise
        encoderPosition--;
    }
}

// PID Setup
const float kp = 0.4;
const float ki = 0.5;
const float kd = 0.0;
const float outMin = -255.0;
const float outMax = 255.0;
const float sampleTime = 0.001;
const float tau = 0.0001;

//Motor angle variables. If there are already variables, remove these and add pre-existing variables to the definition later
double a_motor_angle = 240;
double b_motor_angle = 120;



void setup() {
  Serial.begin(115200);

  // Setup pins as Input/Output
  pinMode(C1_PIN, INPUT_PULLUP);
  pinMode(C2_PIN, INPUT_PULLUP);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(C1_PIN), handleEncoder, RISING);

  // Setup PWM
  ledcAttachPin(M1_PIN, pwmChannel1); // Attach PWM to channel 0
  ledcSetup(pwmChannel1, freq, resolution);

  ledcAttachPin(M2_PIN, pwmChannel2); // Attach PWM to channel 0
  ledcSetup(pwmChannel2, freq, resolution);

  lastTime = millis();
 
  home(); // Call home function to calibrate the arms

  // Setting up PID controller
  pid.initialise(kp, ki, kd, outMin, outMax, sampleTime, tau);

  //Initilaise SPM controller by passing the addresses of the motor angles and the z_angle (always 36 but can be edited for modularity)
  spm.begin(&a_motor_angle, &b_motor_angle);
  spm.calculate_motors(0,0);
  Serial.println(a_motor_angle);
  Serial.println(b_motor_angle);
  //
}

void loop() {
  // Testing PID
  float setpoint = 360 * GYZ; // 120 degrees
  float measurement = encoderPosition;

  float output = pid.move(setpoint, measurement); // Get PID output (value between -255 AND 255)

  // Write output to motor
  if (output > 0) {
    // Clockwise
    ledcWrite(pwmChannel1, 255 - output);
    ledcWrite(pwmChannel2, 255);
  } else {
    // Anti-clockwise
    ledcWrite(pwmChannel1, 255);
    ledcWrite(pwmChannel2, 255 - output * -1);
  }

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