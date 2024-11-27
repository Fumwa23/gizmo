#include "SPMController.h"
SPMController::SPMController() {}


void SPMController::begin(double * a_motor_ptr, double * b_motor_ptr){

    pi = 2*acos(0);
    
    double sz_angle_sin = sin(36*pi/180);
    double cz_angle_cos = cos(36*pi/180);

    vector <double> driver_arm(3);

    vector <double> cMotor = {sz_angle_sin,0,-cz_angle_cos};
}

void SPMController::calculate_motors(double phi, double theta){
  //Find direction vector given angle
  get_direction_vector(20,180);
  print_vector(driver_arm, "DRIVER ARM");
  
  //Find joint c (attached to static motor)
  vector <double> cJoint = cross_product(cMotor, driver_arm);
  print_vector(cJoint, "JOINT C");

  //Get cross part of quaternion rotation
  vector <double> rotationCross = scaxvec(cross_product(driver_arm, cJoint),sqrt(3)/2);
  print_vector(rotationCross, "ROTATION CROSS");

  //Get constant part of vector rotation
  vector <double> rotConst = scaxvec(cJoint, -0.5);
  print_vector(rotConst, "Rotation constant");

  //Get A joint position
  vector <double> aJoint = sub_vectors(rotConst, rotationCross);
  print_vector(aJoint, "A JOINT");

  //Get B joint position
  vector <double> bJoint = add_vectors(rotConst, rotationCross);
  print_vector(bJoint, "B JOINT");

  //Get A motor position
  double aMotor = get_motor_angle(aJoint);
  Serial.println("MOTOR A ANGLE");
  Serial.println(aMotor);
  Serial.println();

  //Get B motor position
  double bMotor = get_motor_angle(bJoint);
  Serial.println("MOTOR B ANGLE");
  Serial.println(bMotor);
  Serial.println();

  * a_motor_ptr = aMotor;
  * b_motor_ptr = bMotor;
}




//Function to get the unit vector of the driver arm with a given angle, where theta is angle around the z axis and phi is the angle from the z axis
void SPMController::get_direction_vector(double phi, double theta)
{
    double rphi_rads = phi*pi/180;
    double rtheta_rads = theta*pi/180;
    driver_arm[0] = sin(rphi_rads)*cos(rtheta_rads);
    driver_arm[1] = sin(rphi_rads)*sin(rtheta_rads);
    driver_arm[2] = cos(rphi_rads);
}

//Function to get the angle of the joint vector in the xy plane
double SPMController::get_joint_angle(double x,double y){
  double angle;
  //check the quadrant of the angle to increment it the correct amount
  if (x<0){
    angle = atan(y/x) + pi;
  }else if (x>0){
    if (y<0){
      angle = atan(y/x)+2*pi;
    }else{
      angle = atan(y/x);
    }
  //Remove cases where x = 0 to prevent dividing by 0
  }else{
    if (y > 0){
      angle = pi/2;
    }else{
      angle = 3*pi/2;
    }
  }
  return angle;
}

//Function to solve for the motor angle from the x axis, knowing the vector of the joint
double SPMController::get_motor_angle(vector <double> joint){
  //Define parameters of equation mcosx + nsinx = p where x is the desired angle
  double m = joint[0]*sz_angle_sin;
  double n = joint[1]*sz_angle_sin;
  double p = joint[2]*cz_angle_cos;
  //Define gamma, such that cos(x-gamma) = p/sqrt(n^2+m^2)
  double gamma = get_joint_angle(joint[0],joint[1]);

  //Define q, where q is p/sqrt(m^2+n^2)
  double q = p/sqrt(m*m+n*n);
  //Generate angle
  double motor_angle = pi-acos(q)+gamma;
  if (motor_angle > 2*pi){
    motor_angle -= 2*pi;
  }
  //Convert to degrees
  return motor_angle/pi*180;
}




//Function to calculate the cross product of 2 vectors
vector <double> SPMController::cross_product(vector<double> a, vector <double> b){
  vector <double> c(3); // Initialize vector with 3 elements
  c[0] = a[1]*b[2]-a[2]*b[1];
  c[1] = a[2]*b[0]-a[0]*b[2];
  c[2] = a[0]*b[1]-a[1]*b[0];
  return c;
}


//Function to multiply a vector by a scalar
vector <double> SPMController::scaxvec(vector <double> vec, double sca){
  vector <double> new_vec(3);
  int i;
  for (i=0; i<vec.size();i++){
    new_vec[i] = vec[i]*sca;
  }
  return new_vec;
}


//Function to add 2 vectors
vector <double> SPMController::add_vectors(vector <double> a, vector <double> b){
  vector <double> new_vec(3);
  int i;
  for (i=0; i<a.size();i++){
    new_vec[i] = a[i]+b[i];
  }
  return new_vec;
}


//Function to subtract a vector from another
vector <double> SPMController::sub_vectors(vector <double> a, vector <double> b){
  vector <double> new_vec(3);
  int i;
  for (i=0; i<a.size();i++){
    new_vec[i] = a[i]-b[i];
  }
  return new_vec;
}

//Function to serial print a vector
void SPMController::print_vector(vector <double> to_print, String title ){
  int i;
  Serial.println(title);
  for (i=0; i<to_print.size(); i++){
    Serial.println(to_print[i]);
  }
  Serial.println();
}

