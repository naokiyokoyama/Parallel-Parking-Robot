/*
  L298N.cpp - Library for utilizing the L298 H-bridge.
  Created September 5th, 2014.
  Naoki Yokoyama
*/

#include "Arduino.h"
#include "L298N.h"

L298::L298() {}

//void L298::assignPins(int EnablePin, int Input1Pin, int Input2Pin) {
//  enablePin = EnablePin;
//  input1Pin = Input1Pin;
//  input2Pin = Input2Pin;
//}

void L298::assignPins(int Input1Pin, int Input2Pin) {
  input1Pin = Input1Pin;
  input2Pin = Input2Pin;
}

//void L298::move(int power){
//  bool direction = false;
//  if(power<0)
//    direction = true;
//  digitalWrite(input1Pin, direction);
//  digitalWrite(input2Pin, !direction);
//  analogWrite(enablePin, abs(power));
//}

void L298::move(int power){
  bool direction = false;
  if(power<0)
    direction = true;
  if(direction) {
    digitalWrite(input1Pin, LOW);
    analogWrite(input2Pin, abs(power));
  } else {
    digitalWrite(input2Pin, LOW);
    analogWrite(input1Pin, abs(power));
  }
}

void L298::brake(){
  digitalWrite(input1Pin, HIGH);
  digitalWrite(input2Pin, HIGH);
  analogWrite(enablePin, 0);
}
