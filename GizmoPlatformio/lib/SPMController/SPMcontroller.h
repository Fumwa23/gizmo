#ifndef SPM_CONTROLLER_H
#define SPM_CONTROLLER_H

#include <Arduino.h>
#include <math.h>
#include <vector>
using namespace std;

class SPMController{
public:
    SPMController();              // Constructor
    void begin(double * a_motor_ptr, double * b_motor_ptr);
    void calculate_motors(double phi, double theta);       // Example function

private:
    double pi;
    double sz_angle_sin;
    double cz_angle_cos;
    
    vector <double> driver_arm;                 // Private member variable
    vector <double> cMotor;
    
    double * a_motor_ptr;
    double * b_motor_ptr;

    void get_direction_vector(double phi, double theta);
    double get_joint_angle(double x, double y);
    double get_motor_angle(vector <double> joint);
    vector <double> cross_product(vector <double> a, vector <double> b);
    vector <double> scaxvec(vector <double> vec, double sca);
    vector <double> add_vectors(vector <double> a, vector <double> b);
    vector <double> sub_vectors(vector <double> a, vector <double> b);
    void print_vector(vector <double> to_print, String title);
};

#endif // DC_MOTOR_CONTROL_H