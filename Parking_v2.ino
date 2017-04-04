#include <Servo.h>
#include "ITG3200.h"
#include "HMC5883L.h"
#include "L298N.h"
#include "NewPing.h"
#include "PID.h"

// Left Motor
#define EN1  10
#define OUT1 11
#define OUT2 12
// Right Motor
#define EN2  9
#define OUT3 14
#define OUT4 13
L298 left(EN1,OUT1,OUT2), right(EN2,OUT3,OUT4);
// Front Servo
#define FSERVO 8
#define FLATVAL_F 1540
#define RVAL_F 700
Servo front;
// Back Servo
#define BSERVO 15
#define FLATVAL_B 1025
#define RVAL_B 1950
Servo back;
// Front Sonar
#define FSONAR_TRIG 22
#define FSONAR_ECHO 21
#define MAX_DISTANCE 200
NewPing fsonar(FSONAR_TRIG,FSONAR_ECHO,MAX_DISTANCE);
// Back Sonar
#define BSONAR_TRIG 16
#define BSONAR_ECHO 17
NewPing bsonar(BSONAR_TRIG,BSONAR_ECHO,MAX_DISTANCE);
// Left Encoder
#define LENCODER 3  
// Right Encoder
#define RENCODER 2

volatile int lpulse, rpulse;
volatile unsigned long ltimer, rtimer,lcount,rcount;

float heading,dps,prevdps,init_heading;                                                
unsigned long timer;
int fangle, bangle;

void risingTrig_l() {
  ltimer = micros();
  lcount++;
  attachInterrupt(LENCODER, fallingTrig_l, FALLING);
}

void fallingTrig_l() {
  lpulse = micros() - ltimer;
  attachInterrupt(LENCODER, risingTrig_l, RISING);
}

void risingTrig_r() {
  rtimer = micros();
  rcount++;
  attachInterrupt(RENCODER, fallingTrig_r, FALLING);
}

void fallingTrig_r() {
  rpulse = micros() - rtimer;
  attachInterrupt(RENCODER, risingTrig_r, RISING);
}

void setup() {
  Serial.begin(115200);
  attachInterrupt(LENCODER, risingTrig_l, RISING);
  attachInterrupt(RENCODER, risingTrig_r, RISING);
  Wire.begin();
  init_itg();
  init_mag();
//  magCalibrate();
  while(1) {
    Serial.println(bsonar.read_cm());
    delay(10);
  }
  front.attach(FSERVO);
  back.attach(BSERVO);
  heading = getHeading();
  init_heading = getHeading();
  driveToSpot();
  pivotRight(35);
  frontBump();
  Serial.println("1");
  pivotSpin(-35);
  Serial.println("2");
  backBump();
  Serial.println("3");
  pivotSpin(-35);
  Serial.println("4");
  frontBump();
  left.brake();
  right.brake();
}

void driveToSpot() {
  static unsigned long timer = millis();
  int base;
  bool accel = 0;
  while(1) {
    if(!accel && base == 80) {
      accel = 1;
      lcount = 0;
    }
    updateHeading();
    ctrlFrontServo(90);
    ctrlBackServo(90);
    base = (millis() - timer)/150*5;
    base = constrain(base,0,80);
    driveStraight(base,0.0);
    unsigned long cm = fsonar.read_cm();
    if(cm>15 && accel && lcount>12) {
      Serial.println(lcount);
      return;
    }
  }
}

float getHeading2() {
  return getHeading() - init_heading;
}

void loop() {}

void updateHeading() {
  unsigned long dt = micros()-timer;
  timer = micros();
  float prevdps = dps;
  dps = (float)-readZ() / 14.375;
  float trapezoid = (prevdps+dps)*(float)dt*0.0000005;
  heading = (heading+trapezoid)*0.98+getHeading2()*0.02;
}

void driveStraight(int base,float goal) {
  int val = imuDrive.compute(heading-goal,dps);
  int lpow = base - val;
  int rpow = base + val;
  left.move(lpow);
  right.move(rpow);
}

void pivotRight(float goal) {
  while(abs(heading-goal)>2) {
    ctrlFrontServo(0);
    ctrlBackServo(0);
    updateHeading();
    int val = imuDrive.compute(heading-goal,dps);
    int lpow = -1.6*val;
    int rpow = 0;
    left.move(lpow);
    right.move(rpow);
  }
}

void pivotSpin(float goal) {
  while(abs(heading-goal)>2) {
    updateHeading();
    ctrlFrontServo(0);
    ctrlBackServo(0);
    int val = imuDrive.compute(heading-goal,dps);
    int lpow = -val;
    int rpow = val;
    left.move(lpow);
    right.move(rpow);
  }
}

void frontBump() {
  float orig_heading=heading;
  lcount = 0;
  int base = 45;
  unsigned long timer = millis();
  while(1) {
    while(fsonar.read_cm()>8) {
      updateHeading();
      ctrlFrontServo(0);
      ctrlBackServo(0);
      driveStraight(base,orig_heading);
      if(micros()>ltimer+400000) {
        ltimer = micros();
        base+=8;
      }
    }
    left.brake();
    right.brake();
    if(micros()>ltimer+1000000 && lcount>5)
      return;
  }
}

void backBump() {
  float orig_heading=heading;
  lcount = 0;
  int base = -45;
  unsigned long timer = millis();
  while(1) {
    while(bsonar.read_cm()>8) {
      updateHeading();
      ctrlFrontServo(0);
      ctrlBackServo(0);
      driveStraight(base,orig_heading);
      if(micros()>ltimer+400000) {
        ltimer = micros();
        base-=8;
      }
    }
    left.brake();
    right.brake();
    if(micros()>ltimer+1000000 && lcount>1)
      return;
  }
}

// front > = CCW
// back > = CCW
// CW = <
void ctrlFrontServo(int goal) {
  int fangle = map(goal - heading,0,90,FLATVAL_F,RVAL_F);
  front.writeMicroseconds(fangle);
}

void ctrlBackServo(int goal) {
  int bangle = map(goal + heading,0,90,FLATVAL_B,RVAL_B);
  back.writeMicroseconds(bangle);
}
