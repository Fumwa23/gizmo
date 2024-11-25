#include "PIDController.h"

PIDController::PIDController(): 
kp_(0.0f), ki_(0.0f), kd_(0.0f), tau_(0.0f), 
integrator_(0.0f), prevError_(0.0f), 
differentiator_(0.0f), prevMeasurement_(0.0f), 
outMax_(0.0f), outMin_(0.0f), output_(0.0f)
{}

void PIDController::initialise(float kp, float ki, float kd, float outMin, float outMax, float sampleTime) {

    // Set gains
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    // Set output limits
    outMin_ = outMin;
    outMax_ = outMax;

    // Set sample time
    tau_ = sampleTime;

    // Reset memory
    integrator_ = 0.0f;
    prevError_ = 0.0f;

    differentiator_ = 0.0f;
    prevMeasurement_ = 0.0f;

    output_ = 0.0f;
}

float PIDController::move(float setpoint, float measurement) {
    // Calculate error
    float error = setpoint - measurement;

    // Calculate proportional term
    float proportional = kp_ * error;

    // Calculate integral term
    integrator_ += ki_ * error;

    // Limit integrator
    if (integrator_ > outMax_) {
        integrator_ = outMax_;
    } else if (integrator_ < outMin_) {
        integrator_ = outMin_;
    }

    // Calculate derivative term
    differentiator_ = -kd_ * (measurement - prevMeasurement_) / tau_;

    // Calculate output


}