#include "SPMControllerOwen.h"
SPMControllerOwen::SPMControllerOwen():
pi(2*acos(0)),
    
SIN_36(sin(36*pi/180)),
COS_36(cos(36*pi/180)),
cMotor({SIN_36,0,-COS_36})
{}


void SPMControllerOwen::begin(float * pMotorA, float * pMotorB){
  this-> pMotorA = pMotorA;
  this-> pMotorB = pMotorB;
}

/**
 * @brief Function to calculate the Motor angles given the desired angle of the driver arm
 * 
 * @param phi Angle from the z axis
 * @param theta Angle around the z axis
 */
void SPMControllerOwen::calculate_motors(float phi, float theta){
  //Find direction vector given angle
  vector <float> driverArm = get_direction_vector(phi,theta);
  //print_vector(driverArm, "DRIVER ARM"); // Debug
  //Find joint c (attached to static motor)
  vector <float> cJoint = cross_product(cMotor, driverArm);
  //print_vector(cJoint, "JOINT C"); // Debug 


  //Get cross part of quaternion rotation
  vector <float> rotationCross = scaxvec(cross_product(driverArm, cJoint),sqrt(3)/2);
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
  float aMotor = get_motor_angle_a(aJoint);
  // Serial.println("MOTOR A ANGLE");
  // Serial.println(aMotor);
  // Serial.println();

  //Get B motor position
  float bMotor = get_motor_angle_b(bJoint);
  // Serial.println("MOTOR B ANGLE");
  // Serial.println(bMotor);
  // Serial.println();
  
  * pMotorA = aMotor;
  * pMotorB = bMotor;
}




//Function to get the unit vector of the driver arm with a given angle, where theta is angle around the z axis and phi is the angle from the z axis
vector <float> SPMControllerOwen::get_direction_vector(float phi, float theta)
{
  vector <float> driverArm(3);
    float rphi_rads = phi*pi/180;
    float rtheta_rads = theta*pi/180;

    driverArm[0] = sin(rphi_rads)*cos(rtheta_rads);
    driverArm[1] = sin(rphi_rads)*sin(rtheta_rads);
    driverArm[2] = cos(rphi_rads);
    return driverArm;
}

//Function to get the angle of the joint vector in the xy plane
float SPMControllerOwen::get_joint_angle(float x,float y){
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
float SPMControllerOwen::get_motor_angle_a(vector <float> joint){
  //Define parameters of equation mcosx + nsinx = p where x is the desired angle
  float m = joint[0]*SIN_36;
  float n = joint[1]*SIN_36;
  float p = joint[2]*COS_36;

  //Solve simultaneous equation to get to solutions (x1, y1), (x2, y2)
  // terms in quadratic formula
  float a = m * m - 1;
  float b = (2 * p * COS_36 * m) / (n * n);
  float c = SIN_36*SIN_36 - (p * p * COS_36 * COS_36) / (n * n);

  vector <float> solutions = quadratic_solver(a, b, c);
  float x1 = solutions[0];
  float x2 = solutions[0]; //TODO change t 1

  // the a solution is always with the smallest x value
  float x = x1 < x2 ? x1 : x2;
  float y = (-p*COS_36 - m*x) / n;

  // Account for different quadrants of solutions:
  if (x>0 && y>0){
    return atan(y/x)*180/pi;
  } else if (x>0 && y<0){
    return 360 + atan(y/x)*180/pi;
  } else if (x<0){
    return 180 + atan(y/x)*180/pi;
  } else {
    return 0.0;
  }
}

//Function to solve for the motor angle from the x axis, knowing the vector of the joint
float SPMControllerOwen::get_motor_angle_b(vector <float> joint){
  //Define parameters of equation mcosx + nsinx = p where x is the desired angle
  float m = joint[0]*SIN_36;
  float n = joint[1]*SIN_36;
  float p = joint[2]*COS_36;

  //Solve simultaneous equation to get to solutions (x1, y1), (x2, y2)
  // terms in quadratic formula
  float a = m * m - 1;
  float b = (2 * p * COS_36 * m) / (n * n);
  float c = SIN_36*SIN_36 - (p * p * COS_36 * COS_36) / (n * n);

  vector <float> solutions = quadratic_solver(a, b, c);
  float x1 = solutions[0];
  float x2 = solutions[1]; //TODO change t 1

  // Find the y solutions for each x value
  float y1 = (-p*COS_36 - m*x1) / n;
  float y2 = (-p*COS_36 - m*x2) / n;

  // the a solution is always with the largest y value
  float x = y1 > y2 ? x1 : x2;
  float y = y1 > y2 ? y1 : y2;

  // Account for different quadrants of solutions:
  if (x>0 && y>0){
    return atan(y/x)*180/pi;
  } else if (x>0 && y<0){
    return 360 + atan(y/x)*180/pi;
  } else if (x<0){
    return 180 + atan(y/x)*180/pi;
  } else {
    return 0.0;
  }
}

/**
 * @breif Function to solve a quadratic equation of the form ax^2 + bx + c = 0
 * This function returns a vector with the 2 solutions to the quadratic equation
 * 
 */
vector <float> SPMControllerOwen::quadratic_solver(float a, float b, float c){
  vector <float> solutions(2);
  float discriminant = b*b-4*a*c;
  if (discriminant < 0){
    solutions[0] = 0;
    solutions[1] = 0;
  }else{
    solutions[0] = (-b+sqrt(discriminant))/(2*a);
    solutions[1] = (-b-sqrt(discriminant))/(2*a);
  }
  return solutions;
}

//Function to calculate the cross product of 2 vectors
vector <float> SPMControllerOwen::cross_product(vector<float> a, vector <float> b){
  vector <float> c(3); // Initialize vector with 3 elements
  c[0] = a[1]*b[2]-a[2]*b[1];
  c[1] = a[2]*b[0]-a[0]*b[2];
  c[2] = a[0]*b[1]-a[1]*b[0];
  return c;
}


/**
 * @brief Function to scale a vector by a scalar
 * 
 * @param vec The vector to be scaled
 * @param sca The scalar to scale the vector by
 */
vector <float> SPMControllerOwen::scaxvec(vector <float> vec, float sca){
  vector <float> new_vec(3);
  int i;
  for (i=0; i<vec.size();i++){
    new_vec[i] = vec[i]*sca;
  }
  return new_vec;
}


//Function to add 2 vectors
vector <float> SPMControllerOwen::add_vectors(vector <float> a, vector <float> b){
  vector <float> new_vec(3);
  int i;
  for (i=0; i<a.size();i++){
    new_vec[i] = a[i]+b[i];
  }
  return new_vec;
}


//Function to subtract a vector from another
vector <float> SPMControllerOwen::sub_vectors(vector <float> a, vector <float> b){
  vector <float> new_vec(3);
  int i;
  for (i=0; i<a.size();i++){
    new_vec[i] = a[i]-b[i];
  }
  return new_vec;
}

//Function to serial print a vector
void SPMControllerOwen::print_vector(vector <float> toPrint, String title ){
  int i;
  Serial.println(title);
  for (i=0; i<toPrint.size(); i++){
    Serial.println(toPrint[i]);
  }
  //Serial.println();
}

