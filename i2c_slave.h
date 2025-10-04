#ifndef __I2C_SLAVE_H_
#define __I2C_SLAVE_H_
#include <Wire.h>

volatile uint8_t x = 0;
volatile uint8_t y = 0;
bool data_received = false;

void I2C_RxHandler(int num_bytes)
{
  //Serial.println("Interrupt");
	while(Wire.available())
	{
		x = Wire.read();
		y = Wire.read();
    data_received = true;
	}
}

void i2c_setup()
{
	Wire.begin(0x55);
	Wire.onReceive(I2C_RxHandler);
  Serial.println("I2C Setup");
}
		
#endif