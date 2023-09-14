#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int pwm_f, int pwm_b) {
  pinMode(pwm_f, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  Motor::pwm_f = pwm_f; //forward pwm
  Motor::pwm_b = pwm_b; //backward pwm
}

void Motor::rotate(int value) {
  if (value >= 1500) {
    int out = map(value, 1500, 2000, 0, 255);
    analogWrite(pwm_b, 0);
    delay(1); //OCD delay for making sure h bridge does not fry
    analogWrite(pwm_f, out);
  }
  else {
    int out = map(value, 1500, 1000, 0, 255);
    analogWrite(pwm_f, 0);
    delay(1); //OCD delay for making sure h bridge does not fry
    analogWrite(pwm_b, out);
  }
}
