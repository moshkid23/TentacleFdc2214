#ifndef Pid_h
#define Pid_h
#include "Arduino.h" 

class Pid {
public:

  struct DiPo{
  int D;
  int P;
  };

//  DiPo s;
  
  Pid();
  DiPo DirAndPwr(int target, int posi, float kp, float ki, float kd);
  
private:

//pid Variables
  long prevT = 0;
  float eprev = 0;
  float eintegral = 0;

};
#endif
