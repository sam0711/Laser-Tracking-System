#ifndef __I2C_MASTER_H_
#define __I2C_MASTER_H_

#include <Wire.h>

//arduino slave addr is 0x55, refer to following link if it isn't right
//https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/

//uint8_t data[2] = {0,0};

void i2c_setup()
{
	bool status = Wire.begin(); //default pins
  if(!status)
  {
    Serial.println("I2C setup fail");
    while(1);
  }
}

void send_data(uint8_t x, uint8_t y)
{
	//data[0] = x;
	//data[1] = y;
	
	Wire.beginTransmission(0x55);
	Wire.write(x);
  Wire.write(y);
	Wire.endTransmission();
}

#endif
