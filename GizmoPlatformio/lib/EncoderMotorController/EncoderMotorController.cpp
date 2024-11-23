#include "EncoderMotorController.h"

// Constructor: Initialise pins and variables
EncoderMotorController::EncoderMotorController(uint8_t pinM1, uint8_t pinM2, uint8_t pinC1, uint8_t pinC2)
    : _pinM1(pinM1), _pinM2(pinM2), _pinC1(pinC1), _pinC2(pinC2),
      _currentSpeed(0), _currentPosition(0), _targetPosition(0),
      _acceleration(0), _kp(1.0), _ki(0.0), _kd(0.0),
      _prevError(0), _integral(0), _lastUpdateTime(0),
      _channelM1(0), _channelM2(1) { // Using PWM channels 0 and 1 for motor control

    // Set up motor control pins for PWM using LEDC
    ledcSetup(_channelM1, 5000, 8); // 5 kHz frequency, 8-bit resolution
    ledcSetup(_channelM2, 5000, 8);

    ledcAttachPin(_pinM1, _channelM1);
    ledcAttachPin(_pinM2, _channelM2);

    // Attach interrupts for encoder pins
    attachInterrupt(digitalPinToInterrupt(_pinC1), [this]() { readEncoder(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_pinC2), [this]() { readEncoder(); }, CHANGE);
}

// Set motor speed
void EncoderMotorController::setSpeed(int speed) {
    _currentSpeed = constrain(speed, -100, 100); // Speed range: -100 to 100
    applyMotorPower(map(_currentSpeed, -100, 100, -255, 255));
}

// Set target position
void EncoderMotorController::setPosition(long position) {
    _targetPosition = position;
}

// Update PID coefficients
void EncoderMotorController::updatePID(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

// Set acceleration
void EncoderMotorController::setAcceleration(float accel) {
    _acceleration = accel;
}

// Update motor control
void EncoderMotorController::update() {
    unsigned long now = millis();
    float deltaTime = (now - _lastUpdateTime) / 1000.0; // Time in seconds
    _lastUpdateTime = now;

    // Apply acceleration if needed
    if (_currentSpeed != 0) {
        int targetSpeed = map(calculatePID(), -255, 255, -100, 100);
        if (abs(_currentSpeed - targetSpeed) > _acceleration * deltaTime) {
            _currentSpeed += (_currentSpeed < targetSpeed ? 1 : -1) * _acceleration * deltaTime;
        } else {
            _currentSpeed = targetSpeed;
        }
        applyMotorPower(map(_currentSpeed, -100, 100, -255, 255));
    }
}

// Apply PWM to motor pins using ESP32 LEDC
void EncoderMotorController::applyMotorPower(int pwm) {
    if (pwm > 0) {
        ledcWrite(_channelM1, pwm);
        ledcWrite(_channelM2, 0);
    } else if (pwm < 0) {
        ledcWrite(_channelM1, 0);
        ledcWrite(_channelM2, abs(pwm));
    } else {
        ledcWrite(_channelM1, 0);
        ledcWrite(_channelM2, 0);
    }
}

// Read encoder ticks (simplified example)
void EncoderMotorController::readEncoder() {
    _currentPosition++; // Placeholder for actual encoder logic
}

// PID Algorithm for Position Control
long EncoderMotorController::calculatePID() {
    float error = _targetPosition - _currentPosition;
    _integral += error;
    float derivative = error - _prevError;
    _prevError = error;

    return _kp * error + _ki * _integral + _kd * derivative;
}