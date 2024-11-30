#include "projectConfig.h"

// Interrupt function 1
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