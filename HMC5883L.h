#include <Wire.h> //I2C Arduino Library
#include <EEPROM.h>

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

short x_offset, y_offset;
float scale;

void initmag() {
  Wire.begin();
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  
  short x_max = EEPROM.read(0) | (EEPROM.read(1)<<8);
  short x_min = EEPROM.read(2) | (EEPROM.read(3)<<8);
  short y_max = EEPROM.read(4) | (EEPROM.read(5)<<8);
  short y_min = EEPROM.read(6) | (EEPROM.read(7)<<8);
  
  x_offset = (x_max + x_min)/2;
  y_offset = (y_max + y_min)/2;
  scale = ((float)(x_max-x_offset))/((float)(y_max-y_offset));
}

void getMagData(short buf[]) {
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    buf[0] = Wire.read()<<8; //X msb
    buf[0] |= Wire.read(); //X lsb
    buf[2] = Wire.read()<<8; //Z msb
    buf[2] |= Wire.read(); //Z lsb
    buf[1] = Wire.read()<<8; //Y msb
    buf[1] |= Wire.read(); //Y lsb
  }
}

void getFixedMagData(short buf[]) {
  getMagData(buf);
  buf[0] -= x_offset;
  buf[1] -= y_offset;
//  buf[1] *= scale;
}

float getHeading() {
  short buf[3];
  getFixedMagData(buf);
  return atan2((float)buf[1],(float)buf[0])/3.14159265*180.0;
}

void magCalibrate() {
//  pinMode(6,INPUT_PULLUP);
//  if(digitalRead(6))
//    return;
  short x_max, x_min, y_max, y_min;
  short magVals[3];
  getMagData(magVals);
  x_max = magVals[0];
  x_min = magVals[0];
  y_max = magVals[1];
  y_min = magVals[1];
  unsigned long timer = millis() + 10000;
  while(millis() < timer) {
    getMagData(magVals);
    x_max = max(x_max,magVals[0]);
    x_min = min(x_min,magVals[0]);
    y_max = max(y_max,magVals[1]);
    y_min = min(y_min,magVals[1]);
  }
  EEPROM.write(0,x_max & 0xFF);
  EEPROM.write(1,x_max >> 8);
  EEPROM.write(2,x_min & 0xFF);
  EEPROM.write(3,x_min >> 8);
  EEPROM.write(4,y_max & 0xFF);
  EEPROM.write(5,y_max >> 8);
  EEPROM.write(6,y_min & 0xFF);
  EEPROM.write(7,y_min >> 8);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  while(1);
}
