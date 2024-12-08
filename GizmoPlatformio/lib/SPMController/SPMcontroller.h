#ifndef SPM_CONTROLLER_H
#define SPM_CONTROLLER_H

#include <Arduino.h>
#include <math.h>
#include <vector>
using namespace std;

class SPMController{
public:
    SPMController();              // Constructor
    void begin(float * pMotorA, float * pMotorB);
    void calculateMotors(float phi, float theta);

private:
    const float SIN_36;
    const float COS_36;
    
    vector <float> driverArm;
    const vector <float> cMotor;
    
    float * pMotorA;
    float * pMotorB;

    vector <float> getDirectionVector(float phi, float theta);
    float getJointAngle(float x, float y);
    float getMotorAngle(vector <float> joint);
    vector <float> crossProduct(vector <float> a, vector <float> b);
    vector <float> scaleVector(vector <float> vec, float sca);
    vector <float> addVectors(vector <float> a, vector <float> b);
    vector <float> subVectors(vector <float> a, vector <float> b);
    void printVector(vector <float> toPrint, String title);
};

#endif // SPM_CONTROLLER_H