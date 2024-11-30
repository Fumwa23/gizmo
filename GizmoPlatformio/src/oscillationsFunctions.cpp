#include "projectConfig.h"


//Oscillation functions
void startOscillation(int direction, int amplitude){
  dOscillationDirection = direction;
  mOscillationAmplitude = amplitude;

  sOscillationStart = millis();
  oscillating = true;
  spm.calculate_motors(mOscillationAmplitude,dOscillationDirection);
  float target1 = motorAngle1*GYZ;
  float target2 = motorAngle2*GYZ;

  Serial.print("Target 1 : ");
  Serial.print(target1);
  Serial.print("   Target 2 : ");
  Serial.println(target2);

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

    // Debugging logging:
    unsigned long startTime = millis(); // Start time for timeout
    const unsigned long timeout = 100;

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
    float phi = mOscillationAmplitude*cos(2*pi*t/timePeriod);
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

/**
 * @brief A method of oscillating the spindle in a way where the direction and magnitude of the oscillation can be adjusted dynamically. 
 * 
 * @param direction The integer direction of the plane of oscillation. Between 0 and 360 degrees.
 * @param amplitude The integer magnitude of the oscillation. Between 0 and 42 degrees (values over 42 can not be reached).
 * 
 * @todo Test if the asynchronus delay is needed. 
 * @todo Check if amplitude should be a global variable. 
 */
void dynamicOscillation(int direction, int amplitude){ // Direction of oscillation and amplitude of oscillation
  if (millis() > lastOscillationTime+20){

    // Calculate the magnitude of the oscillation that motors should move to.
    float t = fmod(millis()-sOscillationStart,timePeriod); // Time within oscillation period
    float phi = amplitude*cos(2*pi*t/timePeriod); // Magnitude of oscillation
    spm.calculate_motors(phi, direction);
    
    // Move motors to calculated angle
    float calculatedPWM1 = pid1.move(motorAngle1*GYZ, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    float calculatedPWM2 = pid2.move(motorAngle2*GYZ, encoder2Position);
    analogWrite(2, calculatedPWM2);

    // Debugging logs:
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

    // Add manual damping 
    amplitude =- 1;

    lastOscillationTime = millis();
  }
}