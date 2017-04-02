#include "ITG3200.h"
#include "HMC5883L.h"
#include "L298N.h"
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
//#define FSONAR 10
// Back Sonar
//#define BSONAR 15

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // Reset ITG3200
  itgWrite(itgAddress,0x3E,0x80);
  //Set the gyroscope scale for the outputs to +/-2000 degrees per second
  itgWrite(itgAddress, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));
  //Set the sample rate to 100 hz
  itgWrite(itgAddress, SMPLRT_DIV, 9);
  counteroffset();

//  left.assignPins(OUT2,OUT1);
//  right.assignPins(OUT4,OUT3);
  front.attach(FSERVO);
  back.attach(BSERVO);
}

float drh,dps,prevdps;                                                    
unsigned long timer;
int fangle, bangle;

void loop() {
  unsigned long dt = micros()-timer;
  timer = micros();
  float prevdps = dps;
  dps = (float)readZ() / 14.375;
  float trapezoid = (prevdps+dps)*(float)dt*0.0000005;
  drh+=trapezoid;
}

// front > = CCW
// back > = CCW
// CW = <
void ctrlFrontServo(int goal) {
  fangle = FLATVAL_F - drh - goal;
  front.write(fangle);
}

void ctrlBackServo(int goal) {
  bangle = FLATVAL_B - drh - goal;
  back.write(bangle);
}

