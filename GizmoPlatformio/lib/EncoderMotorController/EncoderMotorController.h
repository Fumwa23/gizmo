#ifndef ENCODER_MOTOR_CONTROL_H
#define ENCODER_MOTOR_CONTROL_H

#include <Arduino.h>

class EncoderMotorController {
public:
    EncoderMotorController();              // Constructor
    void begin(int pin);      // Initialise with a pin
    void doSomething();       // Example function

private:
    int _pin;                 // Private member variable
};

#endif // DC_MOTOR_CONTROL_H