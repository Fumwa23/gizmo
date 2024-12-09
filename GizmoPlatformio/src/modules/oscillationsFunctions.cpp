#include "projectConfig.h"

/**
 * @brief Oscillates the pendulum in a constant circular motion.
 */
void circularOscillation(){
    unsigned long circularOscillationTime = millis();

    if (circularOscillationTime - lastTime >= 100){
      dOscillationDirection += 2;
      lastCircularOscillationTime = circularOscillationTime;

      Serial.print("Direction : ");
      Serial.print(dOscillationDirection%360);
      Serial.print(" | Target 1 : ");
      Serial.print(motorAngle1);
      Serial.print(" Target 2 : ");
      Serial.print(motorAngle2);
      Serial.print(" | Encoder 1 : ");
      Serial.print(encoder1Position/GYZ);
      Serial.print(" Encoder 2 : ");
      Serial.println(encoder2Position/GYZ);
    }

    spm.calculateMotors(aOscillationAmplitude,dOscillationDirection);

    // Move motors to calculated angle
    float calculatedPWM1 = pid1.move(motorAngle1*GYZ, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    float calculatedPWM2 = pid2.move(motorAngle2*GYZ, encoder2Position);
    analogWrite(2, calculatedPWM2);
}


/**
 * @brief Oscillates the motors on a singla plane with a varying amplitude.
 * 
 * The amplitude of the oscillation is calculated using the formula:
 * 
 * phi = A sin(2*pi*t/T)
 * 
 * where:
 * - phi is the magnitude of the oscillation
 * - A is the amplitude of the oscillation
 * - t is the time within the oscillation period
 * - T is the time period of the oscillation
 * 
 * The motors are then moved to the calculated angle.
 */
void dynamicallyOscillate(){
  float t = fmod(millis()-sOscillationStart,timePeriod); // Time within oscillation period


  if (millis() - lastOscillationTime > 20){

    // Serial.print("Amplitude: ");
    // Serial.print(aOscillationAmplitude);
    // Serial.print("Time period: ");
    // Serial.println(timePeriod);


    float phi = aOscillationAmplitude*sin(2*PI*t/timePeriod); // Magnitude of oscillation
    spm.calculateMotors(phi, dOscillationDirection);
    
    // Move motors to calculated angle
    float calculatedPWM1 = pid1.move(motorAngle1*GYZ, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    float calculatedPWM2 = pid2.move(motorAngle2*GYZ, encoder2Position);
    analogWrite(2, calculatedPWM2);

    // DEBUGGING
    // Serial.print("t : ");
    // Serial.print(t);
    // Serial.print("   Phi : ");
    // Serial.print(phi);
    // Serial.print("   PWM1 : ");
    // Serial.print(calculatedPWM1);
    // Serial.print("   Encoder1 : ");
    // Serial.print(encoder1Position/GYZ);
    // Serial.print("   Target1  : ");
    // Serial.print(motorAngle1);
    // Serial.print("   PWM2 : ");
    // Serial.print(calculatedPWM2);
    // Serial.print("   Encoder2 : ");
    // Serial.print(encoder2Position/GYZ);
    // Serial.print("   Target2 : ");
    // Serial.println(motorAngle2);

    lastOscillationTime = millis();
  }
}