#ifndef SPM_CONTROLLER_OWEN_H
#define SPM_CONTROLLER_OWEN_H

#include <Arduino.h>
#include <math.h>
#include <vector>
using namespace std;

class SPMControllerOwen{
public:
    SPMControllerOwen();              // Constructor
    void begin(float * a_motor_ptr, float * b_motor_ptr);
    void calculate_motors(float phi, float theta);       // Example function

private:
    double pi;
    float SIN_36;
    float COS_36;
    
    vector <float> driver_arm;                 // Private member variable
    vector <float> cMotor;
    
    float * a_motor_ptr;
    float * b_motor_ptr;

    vector <float> get_direction_vector(float phi, float theta);
    float get_joint_angle(float x, float y);
    float get_motor_angle(vector <float> joint);
    float get_motor_angle_a(vector <float> joint);
    float get_motor_angle_b(vector <float> joint);
    vector <float> quadratic_solver(float a, float b, float c);

    vector <float> cross_product(vector <float> a, vector <float> b);
    vector <float> scaxvec(vector <float> vec, float sca);
    vector <float> add_vectors(vector <float> a, vector <float> b);
    vector <float> sub_vectors(vector <float> a, vector <float> b);
    void print_vector(vector <float> to_print, String title);
};

#endif // SPM_CONTROLLER_OWEN_H