#include "SPMController.h"
#include <math.h>

SPMController::SPMController():
SIN_36(sin(36*PI/180)),
COS_36(cos(36*PI/180)),
cMotor({SIN_36,0,-COS_36})
{}

/**
 * @brief Function to initialize the motor pointers
 * 
 * @param pMotorA Pointer to the motor angle of motor A
 * @param pMotorB Pointer to the motor angle of motor B
 */
void SPMController::begin(float * pMotorA, float * pMotorB){
  this-> pMotorA = pMotorA;
  this-> pMotorB = pMotorB;
}

/**
 * @brief Function to calculate the motor angles given the desired angle of the driver arm
 * 
 * @param phi Angle from the z axis
 * @param theta Angle around the z axis
 * 
 * @note The output is saved to the pointers for motor position.
 */
void SPMController::calculateMotors(float phi, float theta){
  // Serial.print(" ----- Calclulating for phi : ");
  // Serial.print(phi);
  // Serial.print("   and theta : ");
  // Serial.println(theta);

  //Find direction vector given angle
  vector <float> driverArm = getDirectionVector(phi,theta);
  //printVector(driverArm, "DRIVER ARM");

  //Find joint c (attached to static motor)
  vector <float> cJoint = crossProduct(driverArm, cMotor);
  // printVector(cJoint, "JOINT C");

  //Get cross part of quaternion rotation
  vector <float> rotationCross = scaleVector(crossProduct(driverArm, cJoint),sqrt(3)/2);
  //printVector(rotationCross, "ROTATION CROSS");

  //Get constant part of vector rotation
  vector <float> rotConst = scaleVector(cJoint, -0.5);
  //printVector(rotConst, "Rotation constant");

  //Get A joint position
  vector <float> aJoint = subVectors(rotConst, rotationCross);
  //printVector(aJoint, "A JOINT");

  //Get B joint position
  vector <float> bJoint = addVectors(rotConst, rotationCross);
  //printVector(bJoint, "B JOINT");

  //Get A motor position
  float aMotor = getMotorAngle(aJoint);
  //Serial.println("MOTOR A ANGLE");
  //Serial.println(aMotor);
  //Serial.println();

  //Get B motor position
  float bMotor = getMotorAngle(bJoint);
  //Serial.println("MOTOR B ANGLE");
  //Serial.println(bMotor);
  //Serial.println();
  
  * pMotorA = aMotor;
  * pMotorB = bMotor;
}




/**
 * @brief Function to get the unit vector of the driver arm
 * 
 * @param phi Angle from the z axis
 * @param theta Angle around the z axis
 * @return vector<float> The unit vector of the driver arm
 */
vector <float> SPMController::getDirectionVector(float phi, float theta)
{
  vector <float> driverArm(3);
    float rphiRads = phi*PI/180;
    float rthetaRads = theta*PI/180;

    driverArm[0] = sin(rphiRads)*cos(rthetaRads);
    driverArm[1] = sin(rphiRads)*sin(rthetaRads);
    driverArm[2] = cos(rphiRads);
    return driverArm;
}


/**
 * @brief Function to get the angle of the joint vector to the x axis in the xy plane
 * 
 * @param x x component of the vector
 * @param y y component of the vector
 * 
 * @return float The angle of the vector to the x axis
 */
float SPMController::getJointAngle(float x,float y){
  float angle;

  //check the quadrant of the angle to increment it the correct amount
  if (x<0){
    angle = atan(y/x) + PI;
  }else if (x>0){
    if (y<0){
      angle = atan(y/x)+2*PI;
    }else{
      angle = atan(y/x);
    }
  //Remove cases where x = 0 to prevent dividing by 0
  
  }else{
    if (y > 0){
      angle = PI/2;
    }else{
      angle = 3*PI/2;
    }
  }
  return angle;
}


/**
 * @brief Function to solve for the motor angle from the x axis, knowing the vector of the joint
 * 
 * @param joint The vector of the joint
 * 
 * @return float The angle of the motor from the x axis
 */
float SPMController::getMotorAngle(vector <float> joint){
  // //Define parameters of equation mcosx + nsinx = p where x is the desired angle
  // float m = joint[0]*SIN_36;
  // float n = joint[1]*SIN_36;
  // float p = joint[2]*COS_36;
  // //Define gamma, such that cos(x-gamma) = p/sqrt(n^2+m^2)
  // float gamma = getJointAngle(joint[0],joint[1]);

  // //Define q, where q is p/sqrt(m^2+n^2)
  // float q = p/sqrt(m*m+n*n);
  float xyMag = sqrt(joint[0]*joint[0]+joint[1]*joint[1]);
  float elevation = joint[2]/xyMag;
  //Serial.println(elevation);
  float gamma = asin(elevation*COS_36/SIN_36);
  //Serial.println(gamma);
  vector <float> vertical = {0.0, 0.0, 1.0};
  vector <float> tempVector = crossProduct(joint, vertical);
  float theta = getJointAngle(tempVector[0],tempVector[1]);
  // Serial.print(" Theta : ");
  // Serial.print(theta*180/PI);
  // Serial.print("   Gamma : ");
  // Serial.println(gamma*180/PI);
  float motorAngle = theta+gamma;



  // //Generate angle
  // float motor_angle = PI-acos(q)+gamma;
  // if (motor_angle > 2*PI){
  //   motor_angle -= 2*PI;
  // }
  // //Convert to degrees
  return motorAngle/PI*180;
}




/**
 * @brief Function to calculate the cross product of 2 vectors
 * 
 * @param a First vector
 * @param b Second vector
 * 
 * @return vector<float> The cross product of the 2 vectors
 */
vector <float> SPMController::crossProduct(vector<float> a, vector <float> b){
  vector <float> c(3); // Initialize vector with 3 elements
  c[0] = a[1]*b[2]-a[2]*b[1];
  c[1] = a[2]*b[0]-a[0]*b[2];
  c[2] = a[0]*b[1]-a[1]*b[0];
  return c;
}


/**
 * @brief Function to multiply a vector by a scalar
 * 
 * @param vec The vector to be scaled
 * @param sca The scalar to multiply the vector by
 * 
 * @return vector<float> The scaled vector
 */
vector <float> SPMController::scaleVector(vector <float> vec, float sca){
  vector <float> newVec(3);
  int i;
  for (i=0; i<vec.size();i++){
    newVec[i] = vec[i]*sca;
  }
  return newVec;
}


/**
 * @brief Function to add 2 vectors
 * 
 * @param a First vector
 * @param b Second vector
 * 
 * @return vector<float> The sum of the 2 vectors
 */
vector <float> SPMController::addVectors(vector <float> a, vector <float> b){
  vector <float> newVec(3);
  int i;
  for (i=0; i<a.size();i++){
    newVec[i] = a[i]+b[i];
  }
  return newVec;
}


/**
 * @brief Function to subtract a vector from another
 * 
 * @param a First vector
 * @param b Second vector
 * 
 * @return vector<float> First vector minus the second vector
 */
vector <float> SPMController::subVectors(vector <float> a, vector <float> b){
  vector <float> newVec(3);
  int i;
  for (i=0; i<a.size();i++){
    newVec[i] = a[i]-b[i];
  }
  return newVec;
}


/**
 * @brief Function to serial print a vector
 * 
 * @param toPrint The vector to be printed
 * @param title The title to be printed before the vector
 */
void SPMController::printVector(vector <float> toPrint, String title ){
  int i;
  Serial.print(title);
  for (i=0; i<toPrint.size(); i++){
    Serial.print("   ");
    Serial.print(toPrint[i]);
  }
  Serial.print("   XY Angle : ");
  float angle = getJointAngle(toPrint[0], toPrint[1])*180/PI;
  Serial.println(angle);
}
