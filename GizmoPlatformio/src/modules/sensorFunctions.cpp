#include "projectConfig.h"

/**
 * @brief Interpret the pulses from a rotary dial.
 * 
 * This function identifies rising edges of the pulses from the dial and increments a pulse count
 */
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

/**
 * @brief Update the amplitude and direction of the oscillation based on the number of pulses detected.
 */
void updateOscillationParameters(){
    unsigned long currentTime = millis();
    // every 100ms be subtracting a bit from amplitude
    const float resonantTimePeriod = 700;
    timePeriod = resonantTimePeriod;
    const float maxPulseCount = 300;
    const float maxAmplitude = 30;

    if (pulseCount > maxPulseCount){
        pulseCount = maxPulseCount;
    }

    const int stepsTillMax = 100;

    if (currentTime - lastTime > 100){

        if (aOscillationAmplitude > 0){
        aOscillationAmplitude -= (maxAmplitude/stepsTillMax);
        }

        // every 100ms be taking a bit from the pulse count and adding it to amplitude and time period
        if (pulseCount > 0){
        if (aOscillationAmplitude < maxAmplitude){
            aOscillationAmplitude += (maxAmplitude/stepsTillMax)*3;
        }
        
        pulseCount -= 3;
        }

        // dOscillationDirection = (dOscillationDirection + 2) % 360; // Causes plane of oscillation to rotate

        Serial.print("Pulse count: ");
        Serial.print(pulseCount);
        Serial.print(" | Amplitude: ");
        Serial.print(aOscillationAmplitude);
        Serial.print(" | Time period: ");
        Serial.println(timePeriod);

        lastTime = currentTime;
    }
}