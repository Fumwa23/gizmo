/*
This File is working code which tested if the motor and encoder are working properly.

*/

#include <Arduino.h> // Base library for Arduino functions
#include <driver/ledc.h> // Library for LEDC functions

// put function declarations here:
int myFunction(int, int);

void setupFunction();

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

// Define pins
#define C1_PIN 26 // Encoder channel C1
#define C2_PIN 27 // Encoder channel C2
#define MOTOR_PWM_PIN1 12 // H-bridge control pin 1
#define MOTOR_PWM_PIN2 13 // H-bridge control pin 2
#define POT_PIN 32 // Potentiometer pin

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

// Variables for encoder
volatile int encoderCount = 0;
volatile int encoderCount2 = 0;
unsigned long lastTime = 0;
const int ENCODER_PULSES_PER_REV = 700; // Replace with your encoder's PPR

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
  //pinMode(MOTOR_PWM_PIN1, OUTPUT);
  pinMode(MOTOR_PWM_PIN2, OUTPUT);

  ledcAttachPin(MOTOR_PWM_PIN1, pwmChannel); // Attach PWM to channel 0
  ledcSetup(pwmChannel, freq, resolution);

  lastTime = millis();
}

void loop() {
  // Read potentiometer value and map it to PWM range
  int potValue = analogRead(POT_PIN);
  int pwmValue = map(potValue, 0, 4095, 0, 255); // ESP32 ADC gives a range from 0 to 4095

  // Apply PWM signal to control motor speeds
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