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
const uint8_t servo1 = 1;
const uint8_t servo2 = 2;

// Offset
int Yoff = -90;
int Xoff = 45;


// Servo angle limit (in decimal)
int minlim1 = 670; // 1003 middle
int maxlim1 = 1500; // 
int minlim2 = 1720; // 2072 middle
int maxlim2 = 2400; //

int servo1mid = 1003;
int servo2mid = 2072;

// OV2640 Optical FOV is 68.7 degrees
// Pixel to degree mapping ratio (240 pixels -> 48 degrees)
float pixelToDegreeRatio = 48.0 / 240.0;

// Servo precision: Max precision of servo movement in degrees per bit
float servoPrecision = 300.0 / 1024.0;

int spd = 1023;

int lastX = -1, lastY = -1;

uint16_t one_p_gain = 400;
uint16_t one_i_gain = 0;
uint16_t one_d_gain = 0;

uint16_t two_p_gain = 400;
uint16_t two_i_gain = 0;
uint16_t two_d_gain = 0;

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
  if(!dxl.ping(servo1)){
    DEBUG_SERIAL.print("Cannot ping servo 1\n");
    digitalWrite(32, HIGH);
  }
  if(!dxl.ping(servo2)){
    DEBUG_SERIAL.print("Cannot ping servo 2\n");
    digitalWrite(32, HIGH);
  }
  delay(1000);
  DEBUG_SERIAL.println("Servo 1 and 2 ping successful");
  

  // Turn servo x LED to distinguish between x and y:
  delay(1000);
  dxl.ledOn(servo1);


  // Turn off torque when configuring items in EEPROM area
  delay(1000);
  dxl.torqueOff(servo1);
  dxl.torqueOff(servo2);
  // Sets the Operating mode to Position Control mode
  dxl.setOperatingMode(servo1, OP_POSITION); 
  dxl.setOperatingMode(servo2, OP_POSITION);
  dxl.torqueOn(servo1);
  dxl.torqueOn(servo2);
  DEBUG_SERIAL.println("Setting Operating Mode");


  // Set Position PID Gains
  delay(1000);
  dxl.writeControlTableItem(POSITION_P_GAIN, servo1, one_p_gain);
  dxl.writeControlTableItem(POSITION_I_GAIN, servo1, one_i_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, servo1, one_d_gain);
  dxl.writeControlTableItem(POSITION_P_GAIN, servo2, two_p_gain);
  dxl.writeControlTableItem(POSITION_I_GAIN, servo2, two_i_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, servo2, two_d_gain);
  DEBUG_SERIAL.println("Setting PID gains");


  //Making square
  delay(1000);
  Serial.println("making square");
  delay(1000);
  dxl.setGoalPosition(servo1, minlim1);
  delay(1000);
  dxl.setGoalPosition(servoy, minlim2);
  delay(1000);
  dxl.setGoalPosition(servo1, maxlim1);
  delay(1000);
  dxl.setGoalPosition(servoy, maxlim2);
  delay(1000);
  dxl.setGoalPosition(servo1, minlim1);
  delay(1000);
  dxl.setGoalPosition(servo2, minlim2);
  delay(1000);

}

int mapCoordToServoPos(int coordinate, int prev, int id) {
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
  if(id == servo2){
    degrees = (coordinate * pixelToDegreeRatio);
    // if(delta < 0){
    //   servoPosition = (820 + offsety - (degrees / servoPrecision) + Yoff) * 1.01;
    // }
    // else{
    //   servoPosition = 820 + offsety - (degrees / servoPrecision) + Yoff;
    // }
    servoPosition = 820 + offsety - (degrees / servoPrecision) + Yoff;
    // servoPosition = 820 - (degrees / servoPrecision) + Yoff;

    servoPosition = constrain(servoPosition, minlim2, maxlim2);
  }
  else{
    degrees = (coordinate * pixelToDegreeRatio) ;
    // servoPosition = ((degrees / servoPrecision) + 256 + offsetx + 17) * 1.01;
    servoPosition = (degrees / servoPrecision) + 256 + offsetx + 17;
    // servoPosition = (degrees / servoPrecision) + 256 + Xoff;
    servoPosition = constrain(servoPosition, minlim1, maxlim1);
  }
  
  int delta = servoPosition - prev;

  // This prevents jitter of the servo if it is within this range
  if((delta > 3) || (delta < -3)){
    return servoPosition;
  }
  else{
  return prev;
  }
  // return servoPosition;
}

void loop() {
  // Convert coordinates to servo positions
  int servo1Pos;
  int servo2Pos;
  if((x == 0) && (y == 0)){
    servo1Pos = servo1mid;
    servo2Pos = servo2mid;
  }
  else{
    servo1Pos = mapCoordToServoPos(x, lastX, servo1);
    servo2Pos = mapCoordToServoPos(y, lastY, servo2);
  }

  // Move servos to the calculated positions
  dxl.setGoalPosition(servo1, servo1Pos);
  dxl.setGoalPosition(servo2, servo2Pos);
  
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
