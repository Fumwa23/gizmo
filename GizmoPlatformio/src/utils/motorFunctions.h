#ifndef ADD_H
#define ADD_H

void analogWrite(int motorNumber, float inputPWM, bool remap = true); // Custom analogWrite() function
void moveArmsToHome();

#endif