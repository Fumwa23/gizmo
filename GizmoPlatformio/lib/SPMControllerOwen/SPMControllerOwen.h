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
    float get_motor_angle(vector <float> joint);
    float get_motor_angle_a(vector <float> joint);
    float get_motor_angle_b(vector <float> joint);
    vector <float> quadratic_solver(float a, float b, float c);

    vector <float> cross_product(vector <float> a, vector <float> b);
    vector <float> scaxvec(vector <float> vec, float sca);
    vector <float> add_vectors(vector <float> a, vector <float> b);
    vector <float> sub_vectors(vector <float> a, vector <float> b);
    void print_vector(vector <float> toPrint, String title);
};

#endif // SPM_CONTROLLER_OWEN_H