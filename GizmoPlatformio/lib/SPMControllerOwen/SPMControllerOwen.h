#ifndef SPM_CONTROLLER_OWEN_H
#define SPM_CONTROLLER_OWEN_H

#include <Arduino.h>
#include <math.h>
#include <vector>
using namespace std;

class SPMControllerOwen{
public:
    SPMControllerOwen();              // Constructor
    void begin(float * pMotorA, float * pMotorB);
    void calculateMotors(float phi, float theta);       // Example function

private:
    double pi;
    float SIN_36;
    float COS_36;
    
    vector <float> driverArm;                 // Private member variable
    vector <float> cMotor;
    
    float * pMotorA;
    float * pMotorB;

    vector <float> getDirectionVector(float phi, float theta);
    float getJointAngle(float x, float y);
    float getMotorAngle(vector <float> joint);
    float getMotorAngleA(vector <float> joint);
    float getMotorAngleB(vector <float> joint);
    vector <float> quadratic_solver(float a, float b, float c);

    vector <float> crossProduct(vector <float> a, vector <float> b);
    vector <float> scaleVector(vector <float> vec, float sca);
    vector <float> addVectors(vector <float> a, vector <float> b);
    vector <float> subVectors(vector <float> a, vector <float> b);
    void printVector(vector <float> toPrint, String title);
};

#endif // SPM_CONTROLLER_OWEN_H