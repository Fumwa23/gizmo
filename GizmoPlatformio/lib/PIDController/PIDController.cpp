#include "PIDController.h"

PIDController::PIDController(): 
kp_(0.0f), ki_(0.0f), kd_(0.0f), tau_(0.0001f), sampleTime_(0.001f),
integrator_(0.0f), prevError_(0.0f), 
differentiator_(0.0f), prevMeasurement_(0.0f), 
outMax_(0.0f), outMin_(0.0f), output_(0.0f)
{}

void PIDController::initialise(float kp, float ki, float kd, float outMin, float outMax, float sampleTime, float tau) {

    // Set gains
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    // Set output limits
    outMin_ = outMin;
    outMax_ = outMax;

    // Set sample time
    sampleTime_ = sampleTime;

    // Set derivative low-pass filter time constant (tau)
    tau_ = tau;

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
    integrator_ = integrator_ + 0.5f * ki_ * sampleTime_ * (error + prevError_);


    // Anti-wind-up via dynamic integrator clamping
    float outMaxInt, outMinInt;

    // Calulate integrator limits
    if (outMax_ > proportional) {
        outMaxInt = outMax_ - proportional;
    } else {
        outMaxInt = 0.0f;
    }

    if (outMin_ < proportional) {
        outMinInt = outMin_ - proportional;
    } else {
        outMinInt = 0.0f;
    }

    // Clamp integrator
    if (integrator_ > outMaxInt) {
        integrator_ = outMaxInt;
    } else if (integrator_ < outMinInt) {
        integrator_ = outMinInt;
    }

    // Calculate derivative term - using derivitive on measurement 
    differentiator_ = (2.0f * kd_ * (prevMeasurement_ - measurement) + (2.0f * tau_ - sampleTime_) * differentiator_) / (2.0f * tau_ + sampleTime_);

    // Calculate output
    output_ = proportional + integrator_ + differentiator_;

    // Clamp output
    if (output_ > outMax_) {
        output_ = outMax_;
    } else if (output_ < outMin_) {
        output_ = outMin_;
    }

    // Update memory
    prevError_ = error;
    prevMeasurement_ = measurement;

    return output_;

}