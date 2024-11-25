#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController();        // Constructor

    void initialise(float kp, float ki, float kd, float outMin, float outMax, float sampleTime, float tau);    // Initialise with a pin
    float move(float setpoint, float measurement);     // Example function
 
private:
    // PID gains
    float kp_;
    float ki_;
    float kd_; 

    // Derivative low-pass filter time constant
    float tau_;

    // Sampling Time (seconds)
    float sampleTime_;

    // Memory variables
    float integrator_;
    float prevError_;
    float differentiator_;
    float prevMeasurement_;

    // Output limits
    float outMax_;
    float outMin_;

    // Controller output
    float output_;
};

#endif // PID_CONTROLLER_H