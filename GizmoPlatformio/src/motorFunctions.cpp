#include "projectConfig.h"

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

  // The motor requires a minimum pwm to move, this remaps the values 
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
  dOscillationDirection = 180;
  aOscillationAmplitude = 30;

  spm.calculate_motors(aOscillationAmplitude,dOscillationDirection);
  Serial.println(aOscillationAmplitude);
  Serial.println(dOscillationDirection);
  Serial.println(motorAngle1);
  Serial.println(motorAngle2);
  bool waiting = true;
  while (waiting){
    float calculatedPWM1 = pid1.move(motorAngle1*GYZ, encoder1Position); 
    analogWrite(1, calculatedPWM1);
    // Move motor 2 to 240 degrees
    float calculatedPWM2 = pid2.move(motorAngle2*GYZ, encoder2Position);
    analogWrite(2, calculatedPWM2);

    // Check to see if position has been reached
    if (abs(encoder1Position - motorAngle1*GYZ) < 10 && abs(encoder2Position - motorAngle2*GYZ) < 10){
      analogWrite(1, 0.0); // Stop motors 
      analogWrite(2, 0.0); // Stop motors
      Serial.print("---- HOMING COMPLETE ----");
      Serial.print(" | Encoder 1 position: ");
      Serial.print(encoder1Position/GYZ);
      Serial.print(" | Encoder 2 position: ");
      Serial.println(encoder2Position/GYZ);
      waiting = false;
    }

    if (millis() - startTime > timeout) {
      Serial.print("---- WAITING FOR HOME TO COMPLETE ----");
      
      // Print output
      Serial.print(" | Output: ");
      Serial.print(calculatedPWM1);

      Serial.print(" Encoder1Position: ");
      Serial.print(encoder1Position/GYZ);

      // Print output
      Serial.print(" | Output2: ");
      Serial.print(calculatedPWM2);

      Serial.print(" aEncoderPosition: ");
      Serial.print(encoder2Position/GYZ);

      Serial.print("    Waiting : ");
      Serial.println(waiting);

      startTime = millis();
      }
    }
    Serial.println(" ----- STARTING CIRCLE ----- ");
    while (dOscillationDirection<900){
      circularOscillation();
    }
    while (true){
    float calculatedPWM1 = pid1.move(120*GYZ, encoder1Position); 
    analogWrite(1, calculatedPWM1);
    // Move motor 2 to 240 degrees
    float calculatedPWM2 = pid2.move(240*GYZ, encoder2Position);
    analogWrite(2, calculatedPWM2);

    // Check to see if position has been reached
    if (abs(encoder1Position - 120*GYZ) < 10 && abs(encoder2Position - 240*GYZ) < 10){
      analogWrite(1, 0.0); // Stop motors 
      analogWrite(2, 0.0); // Stop motors
      Serial.print("---- HOMING COMPLETE ----");
      Serial.print(" | Encoder 1 position: ");
      Serial.print(encoder1Position/GYZ);
      Serial.print(" | Encoder 2 position: ");
      Serial.println(encoder2Position/GYZ);
      dOscillationDirection = 0;
      aOscillationAmplitude = 0;
      break;
    }

    if (millis() - startTime > timeout) {
      Serial.print("---- WAITING FOR HOME TO COMPLETE ----");
      
      // Print output
      Serial.print(" | Output: ");
      Serial.print(calculatedPWM1);

      Serial.print(" Encoder1Position: ");
      Serial.print(encoder1Position/GYZ);

      // Print output
      Serial.print(" | Output2: ");
      Serial.print(calculatedPWM2);

      Serial.print(" aEncoderPosition: ");
      Serial.print(encoder2Position/GYZ);

      startTime = millis();
    }
  }
}