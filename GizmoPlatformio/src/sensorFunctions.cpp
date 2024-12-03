#include "projectConfig.h"

/**
 * @brief Tracks pulses and adds to pulse count every time one occurs
 *
 * @todo Add section to cause pulses to decreae over time. 
 */
void trackDialPulses(){
     bool restState = digitalRead(REST_PIN);

    if (restState == LOW){ // When the dial is not in rest state.

        bool pulseState = digitalRead(PULSE_PIN);

        if (pulseState && !lastPulseState){
            lastPulseCount = pulseCount;
            pulseCount++;
        }
        //set pulsed to hold previous value for edge detection
        lastPulseState = pulseState;
    }
}

/**
 * @brief Tracks the number dialed on a rotary dial.
 *
 * @todo bug: the "Number dialed" message is printed multiple times.
 */
void trackNumberDialed(){
    bool restState = digitalRead(REST_PIN);

    if (restState == LOW){
        //Dial is not in rest state.
        if (!dialling){
            //Just started dialling
            dialling = true;
        }
        //Read pulse pin
        bool pulseState = digitalRead(PULSE_PIN);

        if (pulseState && !lastPulseState){
            pulseCount++;
            Serial.print("Pulse detected: ");
            Serial.println(pulseCount);
        }
        //set pulsed to hold previous value for edge detection
        lastPulseState = pulseState;
    }else{ 
        //Dial is in rest state. Check if it has just returned
        if (dialling){
            //Just finished dialling
            dialling = false;
            
            //DO SOMETHING
            Serial.print("Number dialed: ");
            Serial.println(pulseCount);

            pulseCount = 0;
        }
  }
}