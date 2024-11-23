#include "EncoderMotorController.h"

EncoderMotorController::EncoderMotorController() {
    // Constructor implementation
}

void EncoderMotorController::begin(int pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
}

void EncoderMotorController::doSomething() {
    digitalWrite(_pin, HIGH);
    delay(1000);
    digitalWrite(_pin, LOW);
    delay(1000);
}