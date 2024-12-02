#include "SPMController.h"
SPMController::SPMController():
pi(2*acos(0)),
    
sz_angle_sin(sin(36*pi/180)),
cz_angle_cos(cos(36*pi/180)),
cMotor({sz_angle_sin,0,-cz_angle_cos})
{}


void SPMController::begin(float * a_motor_ptr, float * b_motor_ptr){
  this-> a_motor_ptr = a_motor_ptr;
  this-> b_motor_ptr = b_motor_ptr;
}

/**
 * @brief Function to calculate the motor angles given the desired angle of the driver arm
 * 
 * @param phi Angle from the z axis
 * @param theta Angle around the z axis
 */
void SPMController::calculate_motors(float phi, float theta){
  //Find direction vector given angle
  vector <float> driver_arm = get_direction_vector(phi,theta);
  //print_vector(driver_arm, "DRIVER ARM");
  //Find joint c (attached to static motor)
  vector <float> cJoint = cross_product(cMotor, driver_arm);
  //print_vector(cJoint, "JOINT C");

  //Get cross part of quaternion rotation
  vector <float> rotationCross = scaxvec(cross_product(driver_arm, cJoint),sqrt(3)/2);
  //print_vector(rotationCross, "ROTATION CROSS");

  //Get constant part of vector rotation
  vector <float> rotConst = scaxvec(cJoint, -0.5);
  //print_vector(rotConst, "Rotation constant");

  //Get A joint position
  vector <float> aJoint = sub_vectors(rotConst, rotationCross);
  //print_vector(aJoint, "A JOINT");

  //Get B joint position
  vector <float> bJoint = add_vectors(rotConst, rotationCross);
  //print_vector(bJoint, "B JOINT");

  //Get A motor position
  float aMotor = get_motor_angle(aJoint);
  //Serial.println("MOTOR A ANGLE");
  //Serial.println(aMotor);
  //Serial.println();

  //Get B motor position
  float bMotor = get_motor_angle(bJoint);
  //Serial.println("MOTOR B ANGLE");
  //Serial.println(bMotor);
  //Serial.println();
  
  * a_motor_ptr = aMotor;
  * b_motor_ptr = bMotor;
}




//Function to get the unit vector of the driver arm with a given angle, where theta is angle around the z axis and phi is the angle from the z axis
vector <float> SPMController::get_direction_vector(float phi, float theta)
{
  vector <float> driver_arm(3);
    float rphi_rads = phi*pi/180;
    float rtheta_rads = theta*pi/180;

    driver_arm[0] = sin(rphi_rads)*cos(rtheta_rads);
    driver_arm[1] = sin(rphi_rads)*sin(rtheta_rads);
    driver_arm[2] = cos(rphi_rads);
    return driver_arm;
}

//Function to get the angle of the joint vector in the xy plane
float SPMController::get_joint_angle(float x,float y){
  float angle;
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
float SPMController::get_motor_angle(vector <float> joint){
  // //Define parameters of equation mcosx + nsinx = p where x is the desired angle
  // float m = joint[0]*sz_angle_sin;
  // float n = joint[1]*sz_angle_sin;
  // float p = joint[2]*cz_angle_cos;
  // //Define gamma, such that cos(x-gamma) = p/sqrt(n^2+m^2)
  // float gamma = get_joint_angle(joint[0],joint[1]);

  // //Define q, where q is p/sqrt(m^2+n^2)
  // float q = p/sqrt(m*m+n*n);
  float xyMag = sqrt(joint[0]*joint[0]+joint[1]*joint[1]);
  float elevation = joint[2]/xyMag;
  Serial.println(elevation);
  float gamma = asin(elevation*cz_angle_cos/sz_angle_sin);
  Serial.println(gamma);
  vector <float> vertical = {0.0,0.0,1.0};
  vector<float> tempVector = cross_product(vertical, joint);
  float motorAngle = gamma + get_joint_angle(tempVector[0], tempVector[1]);

  // //Generate angle
  // float motor_angle = pi-acos(q)+gamma;
  // if (motor_angle > 2*pi){
  //   motor_angle -= 2*pi;
  // }
  // //Convert to degrees
  return motorAngle/pi*180;
}




//Function to calculate the cross product of 2 vectors
vector <float> SPMController::cross_product(vector<float> a, vector <float> b){
  vector <float> c(3); // Initialize vector with 3 elements
  c[0] = a[1]*b[2]-a[2]*b[1];
  c[1] = a[2]*b[0]-a[0]*b[2];
  c[2] = a[0]*b[1]-a[1]*b[0];
  return c;
}


//Function to multiply a vector by a scalar
vector <float> SPMController::scaxvec(vector <float> vec, float sca){
  vector <float> new_vec(3);
  int i;
  for (i=0; i<vec.size();i++){
    new_vec[i] = vec[i]*sca;
  }
  return new_vec;
}


//Function to add 2 vectors
vector <float> SPMController::add_vectors(vector <float> a, vector <float> b){
  vector <float> new_vec(3);
  int i;
  for (i=0; i<a.size();i++){
    new_vec[i] = a[i]+b[i];
  }
  return new_vec;
}


//Function to subtract a vector from another
vector <float> SPMController::sub_vectors(vector <float> a, vector <float> b){
  vector <float> new_vec(3);
  int i;
  for (i=0; i<a.size();i++){
    new_vec[i] = a[i]-b[i];
  }
  return new_vec;
}

//Function to serial print a vector
void SPMController::print_vector(vector <float> to_print, String title ){
  int i;
  Serial.print(title);
  for (i=0; i<to_print.size(); i++){
    Serial.print("   ");
    Serial.print(to_print[i]);
  }
  Serial.print("   XY Angle : ");
  float angle = get_joint_angle(to_print[0], to_print[1])*180/pi;
  Serial.println(angle);
}
