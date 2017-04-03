#include "ITG3200.h"
#include "HMC5883L.h"
#include "L298N.h"
#include "NewPing.h"
#include "PID.h"
#include <Servo.h>

// Left Motor
#define EN1  10
#define OUT1 11
#define OUT2 12
// Right Motor
#define EN2  9
#define OUT3 14
#define OUT4 13
L298 left, right;
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

volatile unsigned int lpulse, rpulse;
volatile unsigned long ltimer, rtimer;

float heading,dps,prevdps,init_heading;                                                
unsigned long timer;
int fangle, bangle;

void risingTrig_l() {
  ltimer = millis();
  attachInterrupt(LENCODER, fallingTrig_l, FALLING);
}

void fallingTrig_l() {
  lpulse = millis() - ltimer;
  attachInterrupt(LENCODER, risingTrig_l, RISING);
}

void risingTrig_r() {
  rtimer = millis();
  attachInterrupt(RENCODER, fallingTrig_r, FALLING);
}

void fallingTrig_r() {
  rpulse = millis() - rtimer;
  attachInterrupt(RENCODER, risingTrig_r, RISING);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  attachInterrupt(LENCODER, risingTrig_l, RISING);
  attachInterrupt(RENCODER, risingTrig_r, RISING);
  // Reset ITG3200
  itgWrite(itgAddress,0x3E,0x80);
  //Set the gyroscope scale for the outputs to +/-2000 degrees per second
  itgWrite(itgAddress, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));
  //Set the sample rate to 100 hz
  itgWrite(itgAddress, SMPLRT_DIV, 9);
  initmag();
//  magCalibrate();
  left.assignPins(EN1,OUT1,OUT2);
  right.assignPins(EN2,OUT3,OUT4);
  front.attach(FSERVO);
  back.attach(BSERVO);
  heading = getHeading();
  init_heading = getHeading();
}

float getHeading2() {
  return getHeading() - init_heading;
}

void loop() {
  unsigned long dt = micros()-timer;
  timer = micros();
  float prevdps = dps;
  dps = (float)-readZ() / 14.375;
  float trapezoid = (prevdps+dps)*(float)dt*0.0000005;
  heading = (heading+trapezoid)*0.98+getHeading2()*0.02;
  ctrlFrontServo(0);
  ctrlBackServo(0);
  int val = PIDcompute(heading-0,dps);
  int lpow = 80 - val;
  int rpow = 80 + val;
  left.move(lpow);
  right.move(rpow);
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

