#include <DynamixelShield.h>
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
#else
  #define DEBUG_SERIAL Serial
#endif
extern volatile uint8_t x;
extern volatile uint8_t y;
extern bool data_received;

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

uint16_t x_p_gain = 60;
uint16_t x_i_gain = 6;
uint16_t x_d_gain = 12;

uint16_t y_p_gain = 72;
uint16_t y_i_gain = 6;
uint16_t y_d_gain = 16;

const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  i2c_setup();
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(servox);
  dxl.ping(servoy);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(servox);
  dxl.setOperatingMode(servox, OP_POSITION);
  dxl.setOperatingMode(servoy, OP_POSITION);
  dxl.torqueOn(servoy);

  // Set Position PID Gains
  dxl.writeControlTableItem(POSITION_P_GAIN, servox, x_p_gain);
  dxl.writeControlTableItem(POSITION_I_GAIN, servox, x_i_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, servox, x_d_gain);
  dxl.writeControlTableItem(POSITION_P_GAIN, servoy, y_p_gain);
  dxl.writeControlTableItem(POSITION_I_GAIN, servoy, y_i_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, servoy, y_d_gain);

  // Making square
  // dxl.setGoalPosition(servox, x_L_lim);
  // delay(50);
  // dxl.setGoalPosition(servoy, y_L_lim);
  // delay(1000);
  // dxl.setGoalPosition(servox, x_R_lim);
  // delay(1000);
  // dxl.setGoalPosition(servoy, y_R_lim);
  // delay(1000);
  // dxl.setGoalPosition(servox, x_L_lim);
  // delay(1000);
  // dxl.setGoalPosition(servoy, y_L_lim);
  // delay(1000);

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
