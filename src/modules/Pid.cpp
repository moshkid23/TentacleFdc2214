#include <Arduino.h>
#include "Pid.h"

Pid::Pid() {
}

struct DiPo{
  int D;
  int P;
  };


Pid::DiPo Pid::DirAndPwr(int target, int posi, float kp, float ki, float kd){

  DiPo s;
  
  //time difference
  long currT = micros();
  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT;

  //error
//  int e = pos - target;
  int e = target - posi;

  //derivative
  float dedt = (e-eprev)/(deltaT);

  //integral
  eintegral = eintegral + e*deltaT;

  //control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  //motor power
  float pwr = fabs(u);
  if(pwr>255){
    pwr = 255;
    }

  
  //motor direction
  int dir = 1;
  if( u < 0 ){
    dir = -1;
    }

  //store previous error
  eprev = e;

  s.D = dir;
  s.P = pwr;
  
  return s;
  
}
