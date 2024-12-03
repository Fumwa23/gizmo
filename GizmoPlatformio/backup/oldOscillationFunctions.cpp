

/**
 * @brief A method of oscillating the spindle in a way where the direction and magnitude of the oscillation can be adjusted dynamically. 
 * 
 * @param direction The integer direction of the plane of oscillation. Between 0 and 360 degrees.
 * @param amplitude The integer magnitude of the oscillation. Between 0 and 42 degrees (values over 42 can not be reached).
 * 
 * @todo Test if the asynchronus delay is needed. 
 * @todo Check if amplitude should be a global variable. 
 */
void dynamicOscillation(){ // Direction of oscillation and amplitude of oscillation
  // Calculate the magnitude of the oscillation that motors should move to.
  float t = fmod(millis()-sOscillationStart,timePeriod); // Time within oscillation period

  // If the time within oscillation is less than 10ms, set the new direction and amplitude
  if (t >= timePeriod-10){ // giving t a buffer in case it isn't exactly zero
    if (newOscillationDirectionBool){
      dOscillationDirection = newOscillationDirection;
      newOscillationDirectionBool = false;
    }

    if (newOscillationAmplitudeBool){
      aOscillationAmplitude = newOscillationAmplitude;
      newOscillationAmplitudeBool = false;
    }

    if (newTimePeriodBool){
      timePeriod = newTimePeriod;
      newTimePeriodBool = false;
    }
  }
  

  if (millis() - lastOscillationTime > 20){
    // TODO: add function which clamps time period?

    Serial.print("Amplitude: ");
    Serial.print(aOscillationAmplitude);
    Serial.print("Time period: ");
    Serial.println(timePeriod);


    float phi = aOscillationAmplitude*sin(2*pi*t/timePeriod); // Magnitude of oscillation
    spm.calculate_motors(phi, dOscillationDirection);
    
    // Move motors to calculated angle
    float calculatedPWM1 = pid1.move(motorAngle1*GYZ, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    float calculatedPWM2 = pid2.move(motorAngle2*GYZ, encoder2Position);
    analogWrite(2, calculatedPWM2);

    // DEBUGGING
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


    spm.calculate_motors(aOscillationAmplitude,dOscillationDirection);


    // Move motors to calculated angle
    float calculatedPWM1 = pid1.move(motorAngle1*GYZ, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    float calculatedPWM2 = pid2.move(motorAngle2*GYZ, encoder2Position);
    analogWrite(2, calculatedPWM2);
}