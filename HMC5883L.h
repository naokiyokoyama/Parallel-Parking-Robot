#include <Wire.h> //I2C Arduino Library

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

void initmag(){
  Wire.begin();
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

void getMagData(short buf[]){
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

