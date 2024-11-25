#include "MotorController.h"

MotorController::MotorController() {
    // Constructor implementation
}

void MotorController::begin(int pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
}

void MotorController::doSomething() {
    digitalWrite(_pin, HIGH);
    delay(1000);
    digitalWrite(_pin, LOW);
    delay(1000);
}