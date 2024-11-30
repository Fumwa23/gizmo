#include "projectConfig.h"

void trackPulses(){
    //Code to read phone dial
    //Read pulse pin
    bool pulseState = digitalRead(PULSE_PIN);
    if (pulseState && !pulsed){
        pulseCount++;
    }
    //set pulsed to hold previous value for edge detection
    pulsed = pulseState;
}


void trackRestAndPulse(){
    //Code to read phone dial
  //Read rest pin
  bool restState = digitalRead(REST_PIN);

  if (restState == HIGH){
    Serial.println("restState is HIGH");
    //Dial is not in rest state.
    if (!dialling){
      //Just started dialling
      dialling = true;
    }
    //Read pulse pin
    bool pulseState = digitalRead(PULSE_PIN);
    if (pulseState && !pulsed){
      pulseCount++;
    }
    //set pulsed to hold previous value for edge detection
    pulsed = pulseState;
  }else{
    //Dial is in rest state. Check if it has just returned
    if (dialling){
      //Just finished dialling
      dialling = false;
      //DO SOMETHING
      pulseCount = 0;
    }
  }
}
