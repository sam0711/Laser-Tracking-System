#include <Dynamixel2Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "i2c_slave.h"
#include "angle_translation.h"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB  
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;  
#else
  #define DEBUG_SERIAL Serial
#endif

#define DXL_BAUD 1000000
#define DEBUG_BAUD 115200

extern volatile uint8_t x;
extern volatile uint8_t y;
extern bool data_received;
const int ledPin = 32;


// Servo IDs
const uint8_t servoy = 1;
const uint8_t servox = 2;

// Offset
int Yoff = -90;
int Xoff = 45;


// Servo angle limit (in decimal)
int x_L_lim = 256; // -30 deg
int x_R_lim = 464; // +30 deg
int y_L_lim = 515; // -45 deg
int y_R_lim = 820; // +45 deg

// Pixel to degree mapping ratio (240 pixels -> 48 degrees)
float pixelToDegreeRatio = 48.0 / 240.0;

// Servo precision: Max precision of servo movement in degrees per bit
float servoPrecision = 300.0 / 1024.0;

int spd = 1023;

int lastX = -1, lastY = -1;

uint16_t x_p_gain = 400;
uint16_t x_i_gain = 0;
uint16_t x_d_gain = 0;

uint16_t y_p_gain = 400;
uint16_t y_i_gain = 0;
uint16_t y_d_gain = 0;

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);


  // put your setup code here, to run once:
  i2c_setup();


  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(DEBUG_BAUD);


  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(DXL_BAUD);
  delay(2000);
  DEBUG_SERIAL.print("Beginning communication. Baud rate: ");
  DEBUG_SERIAL.println(DXL_BAUD);


  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  delay(1000);
  DEBUG_SERIAL.print("DXL Protocol Version: ");
  DEBUG_SERIAL.println(DXL_PROTOCOL_VERSION);


  // Print error if cannot ping servos:
  if(!dxl.ping(servox)){
    DEBUG_SERIAL.print("Cannot ping servo x\n");
    digitalWrite(32, HIGH);
  }
  if(!dxl.ping(servoy)){
    DEBUG_SERIAL.print("Cannot ping servo y\n");
    digitalWrite(32, HIGH);
  }
  delay(1000);
  DEBUG_SERIAL.println("Servo x and y ping successful");
  

  // Turn servo x LED to distinguish between x and y:
  delay(1000);
  dxl.ledOn(servox);


  // Turn off torque when configuring items in EEPROM area
  delay(1000);
  dxl.torqueOff(servox);
  dxl.torqueOff(servoy);
  dxl.setOperatingMode(servox, OP_POSITION);
  dxl.setOperatingMode(servoy, OP_POSITION);
  dxl.torqueOn(servox);
  dxl.torqueOn(servoy);
  DEBUG_SERIAL.println("Setting Operating Mode");


  // Set Position PID Gains
  delay(1000);
  dxl.writeControlTableItem(POSITION_P_GAIN, servox, x_p_gain);
  dxl.writeControlTableItem(POSITION_I_GAIN, servox, x_i_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, servox, x_d_gain);
  dxl.writeControlTableItem(POSITION_P_GAIN, servoy, y_p_gain);
  dxl.writeControlTableItem(POSITION_I_GAIN, servoy, y_i_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, servoy, y_d_gain);
  DEBUG_SERIAL.println("Setting PID gains");


  //Making square
  delay(1000);
  Serial.println("making square");
  delay(1000);
  dxl.setGoalPosition(servox, x_L_lim);
  delay(1000);
  dxl.setGoalPosition(servoy, y_L_lim);
  delay(1000);
  dxl.setGoalPosition(servox, x_R_lim);
  delay(1000);
  dxl.setGoalPosition(servoy, y_R_lim);
  delay(1000);
  dxl.setGoalPosition(servox, x_L_lim);
  delay(1000);
  dxl.setGoalPosition(servoy, y_L_lim);
  delay(1000);

}

void loop() {
  // Convert coordinates to servo positions
  int servoXPosition;
  int servoYPosition;
  if((x == 0) && (y == 0)){
    servoXPosition = 360;
    servoYPosition = 666;
  }
  else{
    servoXPosition = mapCoordinateToServoPosition(x, lastX, servox);
    servoYPosition = mapCoordinateToServoPosition(y, lastY, servoy);
  }

  // Move servos to the calculated positions
  dxl.setGoalPosition(servox, servoXPosition);
  dxl.setGoalPosition(servoy, servoYPosition);
  
  // if(data_received){
  //   Serial.print("x:");
  //   Serial.print(x);
  //   Serial.print(",");
  //   Serial.print("y:");
  //   Serial.print(y);
  //   Serial.print(",");
  //   Serial.print("X_Angle:");
  //   Serial.print(servoXPosition);
  //   Serial.print(",");
  //   Serial.print("Y_Angle:");
  //   Serial.println(servoYPosition);
  //   data_received = false;
  // }

  lastX = x;
  lastY = y;
  // Delay to simulate loop cycle time
  delay(1); // Adjust based on your requirements
}

int mapCoordinateToServoPosition(int coordinate, int prev, int id) {
  // // Convert pixel coordinate to degrees
  // float degrees = coordinate * pixelToDegreeRatio;
  float degrees;
  // // Convert degrees to servo position (0-1023 range)
  // int servoPosition = degrees / servoPrecision;
  int servoPosition;

  // trig translation offset values
  int offsety = trig_offsety(y) / 0.29;
  int offsetx = trig_offsetx(x) / 0.29;

  // Ensure servoPosition is within the range
  if(id == servoy){
    degrees = (coordinate * pixelToDegreeRatio);
    // if(delta < 0){
    //   servoPosition = (820 + offsety - (degrees / servoPrecision) + Yoff) * 1.01;
    // }
    // else{
    //   servoPosition = 820 + offsety - (degrees / servoPrecision) + Yoff;
    // }
    servoPosition = 820 + offsety - (degrees / servoPrecision) + Yoff;
    // servoPosition = 820 - (degrees / servoPrecision) + Yoff;

    servoPosition = constrain(servoPosition, y_L_lim, y_R_lim);
  }
  else{
    degrees = (coordinate * pixelToDegreeRatio) ;
    // servoPosition = ((degrees / servoPrecision) + 256 + offsetx + 17) * 1.01;
    servoPosition = (degrees / servoPrecision) + 256 + offsetx + 17;
    // servoPosition = (degrees / servoPrecision) + 256 + Xoff;
    servoPosition = constrain(servoPosition, x_L_lim, x_R_lim);
  }
  
  int delta = servoPosition - prev;

  if((delta > 3) || (delta < -3)){
    return servoPosition;
  }
  else{
  return prev;
  }
  // return servoPosition;
}
