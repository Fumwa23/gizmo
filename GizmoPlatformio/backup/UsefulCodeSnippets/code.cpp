


void setupFunction() {
  unsigned long startTime = millis(); // Start time for timeout
  const unsigned long timeout = 10;

  int homePosition1 = 60 * GYZ;
  int homePosition2 = 300 * GYZ;

  while (true){
    
    float calculatedPWM1 = pid1.move(homePosition1, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    // Move motor 2 to 240 degrees
    float calculatedPWM2 = pid2.move(homePosition2, encoder2Position);
    analogWrite(2, calculatedPWM2);

    // Check to see if position has been reached
    if (abs(encoder1Position - homePosition1) < 50 && abs(encoder2Position - homePosition2) < 50){
      Serial.println("---- MOVEMENT COMPLETE ----");
      analogWrite(1, 0); // Stop motors 
      analogWrite(2, 0); // Stop motors
      break;
    }

    if (millis() - startTime > timeout) {
      Serial.print("---- DEBUGGING ----");
      
      // Print output
      Serial.print(" | Output: ");
      Serial.print(calculatedPWM1);

      Serial.print(" Encoder1Position: ");
      Serial.print(encoder1Position/GYZ);

      // Print output
      Serial.print(" | Output2: ");
      Serial.print(calculatedPWM2);

      Serial.print(" aEncoderPosition: ");
      Serial.println(encoder2Position/GYZ);

      startTime = millis();
    }
  }
}


//PRINT STATEMENT FOR TESTING

  // Tracking position every second for debugging
  if (currentTime - lastTime >= 1000){

    lastTime = currentTime;

    // Print output
    Serial.print(" | Output: ");
    Serial.print("INSERT PWM OUTPUT");

    Serial.print(" Encoder1Position: ");
    Serial.print(encoder1Position);

    // Print output
    Serial.print(" | Output2: ");
    Serial.print("INSERT PWM OUTPUT");

    Serial.print(" Encoder2Position: ");
    Serial.print(encoder2Position);
    Serial.println(" |");
  }