/*
  L298.h - Library for utilizing the L298 H-bridge.
  Created September 5th, 2014.
  Naoki Yokoyama
*/

#ifndef L298N_h
#define L298N_h

#include "Arduino.h"

class L298 {
  public:
  L298(int EnablePin, int Input1Pin, int Input2Pin) {
    enablePin = EnablePin;
    input1Pin = Input1Pin;
    input2Pin = Input2Pin;
    pinMode(EnablePin,OUTPUT);
    pinMode(Input1Pin,OUTPUT);
    pinMode(Input2Pin,OUTPUT);
  }
  void move(int power) {
    bool direction = false;
    if(power<0)
      direction = true;
    digitalWrite(input1Pin, direction);
    digitalWrite(input2Pin, !direction);
    analogWrite(enablePin, abs(power));
  }
  void brake() {
    digitalWrite(input1Pin, HIGH);
    digitalWrite(input2Pin, HIGH);
    analogWrite(enablePin, 0);
  }

  private:
  int enablePin, input1Pin, input2Pin; 
};

#endif
