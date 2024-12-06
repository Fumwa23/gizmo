#include "projectConfig.h"

void trackDialPulses(){
     bool restState = digitalRead(REST_PIN);

    if (restState == LOW){ // restate == LOW when the dial is being moved

        bool pulseState = digitalRead(PULSE_PIN);

        if (pulseState && !lastPulseState){ // Detects rising edge of pulse
            pulseCount += 10;
        }

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

    if (restState == LOW){ // restate == LOW when the dial is being moved
        if (!dialling){
            dialling = true;
        }

        bool pulseState = digitalRead(PULSE_PIN);

        if (pulseState && !lastPulseState){ // Detects rising edge of pulse
            pulseCount++;
            Serial.print("Pulse detected: ");
            Serial.println(pulseCount);
        }

        lastPulseState = pulseState;
    }else{ 

        if (dialling){ // if dialling == true, the dial has just returned to rest state
            dialling = false;
            
            Serial.print("Number dialed: ");
            Serial.println(pulseCount);

            pulseCount = 0;
        }
  }
}