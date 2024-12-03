#include "projectConfig.h"


void getTimes(){
  tPhi = millis()%phiTimePeriod;
}

void checkChanges(){
  float testTime = tPhi%(phiTimePeriod/2);
  if (doneCentre){
    if (testTime > phiTimePeriod/4){
      frequency += get_momentum();
      doneCentre = false;
    }
  }else{
    if (testTime < phiTimePeriod/4){
      amplitude += get_momentum();
      doneCentre = true;
    }
  }
}

int get_momentum(){
  int temp;
  if (cachedMomentum!=0){
    temp = cachedMomentum;
    cachedMomentum = 0;
    return temp;
  }else{
    if (pulseCount>0){
      if (pulseCount >= maxMomentumGain){
        pulseCount -= maxMomentumGain;
        cachedMomentum = maxMomentumGain;
        return maxMomentumGain;
      }else{
        temp = pulseCount;
        pulseCount = 0;
        cachedMomentum = temp;
        return temp;        
      }
    }else{
      return minMomentumGain;
    }
  }
}

void doOscillation(){
  if (frequency = 0){
    //TO DO damp/return to rest and lock motors. Should be seperate function
  }else{
    float time = 2*pi*tPhi/phiTimePeriod;
    float phi = amplitude*sin(time*frequency);
    spm.calculate_motors(phi,0);
    float calculatedPWM1 = pid1.move(motorAngle1, encoder1Position); 
    analogWrite(1, calculatedPWM1);

    // Move motor 2 to 240 degrees
    float calculatedPWM2 = pid2.move(motorAngle2, encoder2Position);
    analogWrite(2, calculatedPWM2);
  }
}