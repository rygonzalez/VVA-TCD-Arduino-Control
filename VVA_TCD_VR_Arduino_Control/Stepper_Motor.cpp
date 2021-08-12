#include "Stepper_Motor.h"

Stepper_Motor::Stepper_Motor(byte A, byte A_bar, byte B, byte B_bar, byte step_delay, bool fwd_limit, bool bkwd_limit){
  this -> A = A;
  this -> B = B;
  this -> A_bar = A_bar;
  this -> B_bar = B_bar;
  this -> fwd_limit = fwd_limit;
  this -> bkwd_limit = bkwd_limit;
  init();
}

void Stepper_Motor::init(){
  motorsOff();
}

void Stepper_Motor::motorsOn() { // turns on motors, to start operating
  pinMode(A, OUTPUT);
  digitalWrite(A, LOW); //?
  pinMode(A_bar, OUTPUT);
  digitalWrite(A_bar, LOW); //?
  pinMode(B, OUTPUT);
  digitalWrite(B, LOW); //?
  pinMode(B_bar, OUTPUT);
  digitalWrite(B_bar, LOW); //?
}

void Stepper_Motor::motorsOff() { // turns off motors, to stop current draw. Useful after moving, or when not operating
  pinMode(A, INPUT);
  digitalWrite(A, HIGH);
  pinMode(A_bar, INPUT);
  digitalWrite(A_bar, HIGH);
  pinMode(B, INPUT);
  digitalWrite(B, HIGH);
  pinMode(B_bar, INPUT);
  digitalWrite(B_bar, HIGH);
}

void Stepper_Motor::stepForward(byte steps) {
  int pos = 0;
  motorsOn();
  for (int its = 0; its < (steps / 4) ; its++) {
    if (fwd_limit) {
      Serial.println("Limit switch detects collision");
      break;
    } else {
      switch (pos){
        case 0:
          pos++;
          digitalWrite(A, HIGH);
          digitalWrite(A_bar, LOW);
          digitalWrite(B, HIGH);
          digitalWrite(B_bar, LOW);
          delayMicroseconds (step_delay);
          break;
        case 1:
          pos++;
          digitalWrite(A, LOW);
          digitalWrite(A_bar, HIGH);
          digitalWrite(B, HIGH);
          digitalWrite(B_bar, LOW);
          delayMicroseconds (step_delay);
          break;
        case 2:
          pos++;
          digitalWrite(A, LOW);
          digitalWrite(A_bar, HIGH);
          digitalWrite(B, LOW);
          digitalWrite(B_bar, HIGH);
          delayMicroseconds (step_delay);
          break;
        case 3:
          pos = 0;
          digitalWrite(A, HIGH);
          digitalWrite(A_bar, LOW);
          digitalWrite(B, LOW);
          digitalWrite(B_bar, HIGH);
          delayMicroseconds (step_delay);
          break;
      }
    }
  }
  Serial.println("Stepping Subroutine Complete");
  motorsOff();
}

void Stepper_Motor::stepBackward(byte steps) {
  int pos = 3;
  motorsOn();
  for (int its = 0; its < (steps / 4) ; its++) {
    if (bkwd_limit) {
      Serial.println("Limit switch detects collision");
      break;
    } else {
      switch (pos){
        case 0:
          pos=3;
          digitalWrite(A, HIGH);
          digitalWrite(A_bar, LOW);
          digitalWrite(B, HIGH);
          digitalWrite(B_bar, LOW);
          delayMicroseconds (step_delay);
          break;
        case 1:
          pos--;
          digitalWrite(A, LOW);
          digitalWrite(A_bar, HIGH);
          digitalWrite(B, HIGH);
          digitalWrite(B_bar, LOW);
          delayMicroseconds (step_delay);
          break;
        case 2:
          pos--;
          digitalWrite(A, LOW);
          digitalWrite(A_bar, HIGH);
          digitalWrite(B, LOW);
          digitalWrite(B_bar, HIGH);
          delayMicroseconds (step_delay);
          break;
        case 3:
          pos--;
          digitalWrite(A, HIGH);
          digitalWrite(A_bar, LOW);
          digitalWrite(B, LOW);
          digitalWrite(B_bar, HIGH);
          delayMicroseconds (step_delay);
          break;
      }
    }
  }
  Serial.println("Stepping Subroutine Complete");
  motorsOff();
}
