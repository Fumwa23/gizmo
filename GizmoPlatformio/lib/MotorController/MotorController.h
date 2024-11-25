#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

class MotorController {
public:
    MotorController();              // Constructor
    void begin(int pin);      // Initialise with a pin
    void doSomething();       // Example function

private:
    int _pin;                 // Private member variable
};

#endif // DC_MOTOR_CONTROL_H