#ifndef ENCODER_MOTOR_CONTROLLER_H
#define ENCODER_MOTOR_CONTROLLER_H

#include <Arduino.h>

class EncoderMotorController {
public:
    // Constructor
    EncoderMotorController(uint8_t pinM1, uint8_t pinM2, uint8_t pinC1, uint8_t pinC2);

    // Public Methods
    void setSpeed(int speed);            // Set motor speed (-100 to 100 for forward/reverse)
    void setPosition(long position);    // Set target position in encoder ticks
    void updatePID(float kp, float ki, float kd); // Update PID values
    void setAcceleration(float accel);  // Set acceleration rate (ticks/sec^2)
    void update();                       // Call this in the loop to handle motor control

private:
    // Private Methods
    void applyMotorPower(int pwm);      // Apply PWM to motor pins using ESP32 LEDC
    void readEncoder();                 // Read encoder ticks
    long calculatePID();                // PID algorithm for position control

    // Motor Control Pins
    uint8_t _pinM1, _pinM2, _pinC1, _pinC2;

    // Motor State Variables
    int _currentSpeed;                  // Current motor speed
    long _currentPosition;              // Current encoder position
    long _targetPosition;               // Desired encoder position
    float _acceleration;                // Acceleration rate
    float _kp, _ki, _kd;                // PID coefficients
    float _prevError, _integral;        // PID state
    unsigned long _lastUpdateTime;      // Timestamp for PID and acceleration

    // PWM Channels for ESP32 LEDC
    int _channelM1;
    int _channelM2;
};

#endif // ENCODER_MOTOR_CONTROLLER_H