//this motor driver class is written for IBT2 or BTS 7960 motor driver

#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  public:
    //Constructor : the 2 pwm pins of ibt2 motor driver
    Motor(int pwm_f, int pwm_b );
    //Spin the motor with a percentage value
    void rotate(int value);
    //Motor Outputs: f is forward and b is backward
    int pwm_f;
    int pwm_b;
};

#endif
