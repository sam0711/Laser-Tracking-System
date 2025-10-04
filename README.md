# Laser-Targeting-System

<img src="Laser_tracking_picture.jpg" width="70%" alt="Laser Targeting System" title="Laser Targeting System">

## Hardware

Boards: OpenRB-150 and ESP32S3

Servos: XL330-M288-T by ROBOTIS

## Setting Up Environment

### OpenRB-150

[OpenRB-150 Manual](https://emanual.robotis.com/docs/en/parts/controller/openrb-150/#install-the-arduino-ide)

Follow Section 6.2 through 6.4 of the manual to set up the environment.

### ESP32S3

INSTALL.txt a TEXT file with information on how to compile and run your code, if you used something outside of what's covered in class / lab (optional)

To compile ESP32 Code on Arduino IDE:
Go to your Arduino IDE --> Arduino --> Preferences --> Additional Boards Manager URLs
Input the following in a newline: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

Click ok.

Tools --> Board --> Board Manager
Search: esp32
Select: author Espressif Systems, version 2.0.9 or above

Tools --> Board --> ESP32 Arduino --> XIAO_ESP32S3

Tools --> PSRAM --> OPIPSRAM

You should now be able to compile, and upload code to the ESP32S3

##

### Tutorial/Tips

When uploading the arduino sketch, MAKE SURE TO UNPLUG RX AND TX before uploading.

After it is uploaded, plug rx tx back in and press reset button to make sure "setup" section is ran.

### Chaniging parameters

For adjusting motor parameters, use the [dynamixel wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)

Parameters such as PID gains, baud rate, torque limit, etc. Will be easily accessible through the wizard. [XL-320 Control Table](https://emanual.robotis.com/docs/en/dxl/x/xl320/#control-table-data-address)

[Protocol 2.0 document](http://support.robotis.com/en/product/actuator/dynamixel_pro/communication/instruction_status_packet.htm) 

1. Connect the USB dongle and CONNECT POWER ADADPTER.
2. (One time setting) go to options and pick protocol 2.0 and select all in "Select port to scan" and hit ok.
3. Press scan in top left corner
4. When the servo shows up on the left column section, hit skip.
5. Change parameters. (Make sure torque is off on the top right section of the wizard when changing parameters.)

### Current angle limit
x axis rotation: 256 - 464 (in decimal) (-30 to 30 degrees) 
y axis rotation: 515 - 820 (in decimal) (-45 to 45 degrees)

### Wiring diagram

Looking from above, with the servo head at the top, wire the left plug of the servo to:

* PIN1: GND
* PIN2: 5 volts
* PIN3: Serial TX

![Dynamixel XL-320 wiring diagram](XL320-wiring.jpg)
