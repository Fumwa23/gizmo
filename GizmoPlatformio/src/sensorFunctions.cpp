#include "projectConfig.h"

void trackDialPulses(){
     bool restState = digitalRead(REST_PIN);

    if (restState == LOW){ // When the dial is not in rest state.

        bool pulseState = digitalRead(PULSE_PIN);

        if (pulseState && !lastPulseState){
            lastPulseCount = pulseCount;
            pulseCount += 10;
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


void dropPulseCount(){
    if (millis()>nextPulseDrop && pulseCount > 0){
        pulseCount -= 10;
        nextPulseDrop = millis() + 1000;

        Serial.print("Pulse count: ");
        Serial.println(pulseCount);
    }
}

void updateParameters(){
    const int maxPulseCount = 300;

    if (pulseCount > maxPulseCount){
        pulseCount = maxPulseCount;
    }

    if (lastPulseCount != pulseCount){
        const float resonantTimePeriod = 700;

        newOscillationAmplitude = pulseCount*20/maxPulseCount; // This mean that at max pulse count, the amplitude will be at 30
        newTimePeriod = 4*resonantTimePeriod - 3*pulseCount*resonantTimePeriod/maxPulseCount; // This means that at max pulse count, the time period will be at resonant frequency

        newTimePeriodBool = true;
        newOscillationAmplitudeBool = true;
    }
}

void testingUpdateParameter(){
    unsigned long currentTime2 = millis();

    if (currentTime2-lastTime2 > 100){
        if (increasingPulseCount){
            lastPulseCount = pulseCount;
            pulseCount += 1;

            Serial.print("Pulse count: ");
            Serial.println(pulseCount);

            if (pulseCount >= 300){
                increasingPulseCount = false;
            }
        } else {
            lastPulseCount = pulseCount;    
            pulseCount -= 1;

            Serial.print("Pulse count: ");
            Serial.println(pulseCount);

            if (pulseCount <= 0){
                increasingPulseCount = true;
            }
        }
        
        lastTime2 = currentTime2;
    }
}