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
    void calculateMotors(float phi, float theta);       // Example function

private:
    // float sz_angle_sin;
    // float cz_angle_cos;
    const float SIN_36;
    const float COS_36;
    
    vector <float> driverArm;                 // Private member variable
    const vector <float> cMotor;
    
    float * pMotorA;
    float * pMotorB;

    vector <float> getDirectionVector(float phi, float theta);
    float getJointAngle(float x, float y);
    float getMotorAngle(vector <float> joint);
    vector <float> cross_product(vector <float> a, vector <float> b);
    vector <float> scaxvec(vector <float> vec, float sca);
    vector <float> add_vectors(vector <float> a, vector <float> b);
    vector <float> sub_vectors(vector <float> a, vector <float> b);
    void print_vector(vector <float> toPrint, String title);
};

#endif // DC_MOTOR_CONTROL_H