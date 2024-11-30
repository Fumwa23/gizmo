#include "projectConfig.h"


//Oscillation functions
void startOscillation(int direction, int magnitude){
  Serial.print(millis());
  Serial.println(" ---- STARTING OSCILATTION ---- ");
  dOscillationDirection = direction;
  mOscillationMagnitude = magnitude;
  sOscillationStart = millis();
  oscillating = true;
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