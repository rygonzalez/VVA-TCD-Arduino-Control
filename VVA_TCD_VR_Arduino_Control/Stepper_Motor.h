#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <Arduino.h>

class Stepper_Motor {
  private:
    byte A;
    byte A_bar;
    byte B;
    byte B_bar;
    byte step_delay;
    byte fwd_limit;
    byte bkwd_limit;
    
    void init();
    void motorsOn();
    void motorsOff();
  public:
    Stepper_Motor(byte A, byte A_bar, byte B, byte B_bar, byte step_delay, bool fwd_limit, bool bkwd_limit);
    void stepForward(byte steps);
    void stepBackward(byte steps);
};
#endif
