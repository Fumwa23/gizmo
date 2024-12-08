#include "projectConfig.h"

void setupPins(){
    pinMode(C1_PIN, INPUT_PULLUP);
    pinMode(C2_PIN, INPUT_PULLUP);
    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);
    
    pinMode(C3_PIN, INPUT_PULLUP);
    pinMode(C4_PIN, INPUT_PULLUP);
    pinMode(M3_PIN, OUTPUT);
    pinMode(M4_PIN, OUTPUT);
    
    pinMode(PULSE_PIN, INPUT_PULLUP);
    pinMode(REST_PIN, INPUT_PULLUP);
}

void setupMotors() {
    // Attach interrupts to their corresponding functions
  attachInterrupt(digitalPinToInterrupt(C1_PIN), handleEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(C3_PIN), handleEncoder2, RISING);

  // Setup PWM channels for motor 1
  ledcAttachPin(M1_PIN, pwmChannel1);
  ledcSetup(pwmChannel1, freq, resolution);

  ledcAttachPin(M2_PIN, pwmChannel2); 
  ledcSetup(pwmChannel2, freq, resolution);

  // Setup PWM channels for motor 2
  ledcAttachPin(M3_PIN, pwmChannel3); // Attach PWM to channel 0
  ledcSetup(pwmChannel3, freq, resolution);

  ledcAttachPin(M4_PIN, pwmChannel4); // Attach PWM to channel 0
  ledcSetup(pwmChannel4, freq, resolution);

  // Setting up PID controllers 
  pid1.initialise(kp, ki, kd, outMin, outMax, sampleTime, tau);
  pid2.initialise(kp, ki, kd, outMin, outMax, sampleTime, tau);

  // Setting up kinematics controller
  spm.begin(&motorAngle2, &motorAngle1);

  // --------------------------------------------- CALLIBRATION AND HOMING
 
  // Starting encoder positions on setup.
  encoder1Position = 33.5*GYZ;
  encoder2Position = (360-33.5)*GYZ;
}