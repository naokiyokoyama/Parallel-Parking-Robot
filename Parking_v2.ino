#include "ITG3200.h"
#include "HMC5883L.h"
#include "L298N.h"
#include "NewPing.h"
#include <Servo.h>

// Left Motor
#define OUT1 11
#define OUT2 12
// Right Motor
#define OUT3 14
#define OUT4 13
L298 left, right;
// Front Servo
#define FSERVO 10
#define FLATVAL_F 100
Servo front;
// Back Servo
#define BSERVO 15
#define FLATVAL_B 49
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
  magCalibrate();
  left.assignPins(OUT2,OUT1);
  right.assignPins(OUT4,OUT3);
  front.attach(FSERVO);
  back.attach(BSERVO);
  heading = getHeading();
  init_heading = getHeading();
//  left.move(255);
//  right.move(255);
//  while(1) {
//    Serial.println(getHeading());
//  }
}

void loop() {
  unsigned long dt = micros()-timer;
  timer = micros();
  float prevdps = dps;
  dps = (float)readZ() / 14.375;
  float trapezoid = (prevdps+dps)*(float)dt*0.0000005;
  heading = (heading-trapezoid)*0.98+getHeading()*0.02;
  ctrlFrontServo(0);
  ctrlBackServo(0);
}

// front > = CCW
// back > = CCW
// CW = <
void ctrlFrontServo(int goal) {
  fangle = FLATVAL_F + heading - goal - init_heading;
  front.write(fangle);
}

void ctrlBackServo(int goal) {
  bangle = FLATVAL_B + heading - goal - init_heading;
  back.write(bangle);
}

