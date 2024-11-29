#ifndef SPM_CONTROLLER_H
#define SPM_CONTROLLER_H

#include <Arduino.h>
#include <math.h>
#include <vector>
using namespace std;

class SPMController{
public:
    SPMController();              // Constructor
    void begin(float * a_motor_ptr, float * b_motor_ptr);
    void calculate_motors(float phi, float theta);       // Example function

private:
    double pi;
    float sz_angle_sin;
    float cz_angle_cos;
    
    vector <float> driver_arm;                 // Private member variable
    vector <float> cMotor;
    
    float * a_motor_ptr;
    float * b_motor_ptr;

    vector <float> get_direction_vector(float phi, float theta);
    float get_joint_angle(float x, float y);
    float get_motor_angle(vector <float> joint);
    vector <float> cross_product(vector <float> a, vector <float> b);
    vector <float> scaxvec(vector <float> vec, float sca);
    vector <float> add_vectors(vector <float> a, vector <float> b);
    vector <float> sub_vectors(vector <float> a, vector <float> b);
    void print_vector(vector <float> to_print, String title);
};

#endif // DC_MOTOR_CONTROL_H