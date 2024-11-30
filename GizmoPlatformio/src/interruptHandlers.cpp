#include "projectConfig.h"

// Unfinshed Flag implementation...
// volatile bool sensor1_triggered = false;
// volatile bool sensor2_triggered = false;

/**
 * @brief Interrupt service routine for motor encoder 1
 * 
 * This function is called whenever the encoder 1 detects a RISING change in state
 * 
 * @todo Implement flags so that no time consuming processes are run by the ISR
 */
void IRAM_ATTR handleEncoder1() {
    int state1 = digitalRead(C1_PIN);
    int state2 = digitalRead(C2_PIN);

    if (state1 != state2) {
        // Clockwise
        encoder1Position++;
    } else {
        // Anti-clockwise
        encoder1Position--;
    }
}

// Interrupt function 2
void IRAM_ATTR handleEncoder2() {
    int state1 = digitalRead(C3_PIN);
    int state2 = digitalRead(C4_PIN);

    if (state1 != state2) {
        // Clockwise
        encoder2Position++;
    } else {
        // Anti-clockwise
        encoder2Position--;
    }
}