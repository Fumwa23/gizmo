#include "projectConfig.h"


//Oscillation functions
void startOscillation(int direction, int amplitude){
  dOscillationDirection = direction;
  aOscillationAmplitude = amplitude;

  sOscillationStart = millis();
  oscillating = true;
  spm.calculate_motors(aOscillationAmplitude,dOscillationDirection);
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
  if (millis() - lastOscillationTime > 20){
    float t = fmod(millis()-sOscillationStart,timePeriod);
    float phi = aOscillationAmplitude*sin(2*pi*t/timePeriod); 
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
void dynamicOscillation(){ // Direction of oscillation and amplitude of oscillation
  // Calculate the magnitude of the oscillation that motors should move to.
  float t = fmod(millis()-sOscillationStart,timePeriod); // Time within oscillation period

  // If the time within oscillation is less than 10ms, set the new direction and amplitude
  if (t <= 10){ // giving t a buffer in case it isn't exactly zero
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
    if (circularOscillationTime - lastTime >= 20){
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

void testingFunction(int direction){
  Serial.print("move 0");

  spm.calculate_motors(32, dOscillationDirection);
    
  // Move motors to calculated angle
  float calculatedPWM1 = pid1.move(motorAngle1*GYZ, encoder1Position); 
  analogWrite(1, calculatedPWM1);

  float calculatedPWM2 = pid2.move(motorAngle2*GYZ, encoder2Position);
  analogWrite(2, calculatedPWM2);

  if (encoder1Position - motorAngle1*GYZ < 50 && encoder2Position - motorAngle2*GYZ < 50){
    Serial.print("move 180");

    //dOscillationDirection = dOscillationDirection == 0 ? 180 : 0;

    analogWrite(1, 0);
    analogWrite(2, 0);
    delay(10000);
  }

  // // DEBUGGING
  //   Serial.print("   PWM1 : ");
  //   Serial.print(calculatedPWM1);
  //   Serial.print("   Encoder1 : ");
  //   Serial.print(encoder1Position/GYZ);
  //   Serial.print("   Target1  : ");
  //   Serial.print(motorAngle1);
  //   Serial.print("   PWM2 : ");
  //   Serial.print(calculatedPWM2);
  //   Serial.print("   Encoder2 : ");
  //   Serial.print(encoder2Position/GYZ);
  //   Serial.print("   Target2 : ");
  //   Serial.println(motorAngle2);

}

void manualCircularOscillation(){

  if (stage == 0){   
    // Move motors to calculated angle
    float calculatedPWM1 = pid1.move(35*GYZ, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    float calculatedPWM2 = pid2.move(120*GYZ, encoder2Position);
    analogWrite(2, calculatedPWM2);

    Serial.print("Encoder 1 : ");
    Serial.print(encoder1Position - 35*GYZ < 50);
    Serial.print(" | Encoder 2 : ");
    Serial.println(encoder2Position - 240*GYZ < 50);

    if (encoder1Position - 35*GYZ < 50 && encoder2Position - 120*GYZ < 50){
      Serial.print("Succeed stage 0");

      analogWrite(1, 0);
      analogWrite(2, 0);

      stage = 1;
      //delay(5000);
    }
  }

  if (stage == 1){
    Serial.println(stage);
    // Move motors to calculated angle
    analogWrite(1, 0);

    float calculatedPWM2 = pid2.move(3000*GYZ, encoder2Position);
    analogWrite(2, calculatedPWM2);

    Serial.print("Encoder 1 : ");
    Serial.print(encoder1Position - 35*GYZ < 50);
    Serial.print(" | Encoder 2 : ");
    Serial.println(abs(encoder2Position - 3000*GYZ) < 50);

    if (abs(encoder1Position - 35*GYZ) < 50 && abs(encoder2Position - 300*GYZ) < 50){
      Serial.print("Succeed stage 1");

      analogWrite(1, 0);
      analogWrite(2, 0);

      //delay(1000);
      stage = 2;
    }
  }

  if (stage == 2) {
    Serial.println(stage);
    // Move motors to calculated angle
    float calculatedPWM1 = pid1.move(270*GYZ, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    analogWrite(2, 0);

    if (abs(encoder1Position - 270*GYZ) < 50 && abs(encoder2Position - 300*GYZ) < 50){
      Serial.print("Succeed stage 2");

      analogWrite(1, 0);
      analogWrite(2, 0);

      //delay(1000);
      stage = 0;
    }
  }
}


void circularOscillationOwen(){
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

    spmOwen.begin(&motorAngle2, &motorAngle1);
    spmOwen.calculate_motors(aOscillationAmplitude,dOscillationDirection);


    // Move motors to calculated angle
    float calculatedPWM1 = pid1.move(motorAngle1*GYZ, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    float calculatedPWM2 = pid2.move(motorAngle2*GYZ, encoder2Position);
    analogWrite(2, calculatedPWM2);
}