#include <math.h>

//distance of object in cm from camera
float obj_distance = 80;

//offset of laser from camera in neutral position in cm
float laser_offset = 5.1;

//value to convert from rad to deg
float convert = 57.29578;

//center why value (pixel value based on resolution)
uint8_t y_center = 120;

//both offsets attempts a simple way to account for laser to camera distance difference
//via treating the laser as the camera for calculations. This is done by subtracting
//the y by the laser offset to create a smaller angle if the object is above the camera
//or adding the laser offset if the ball is below the camera
//Note: if confusing, refer to pictures in project notes from Tyler

//fast atan formula found in research paper at this link
//https://nghiaho.com/?p=997

float fast_atan(float in)
{
	return (3.14 / 4) * in - in * (fabs(in) - 1) * (0.2447 + 0.0663 * fabs(in));
}

//translates the x passed into appropriate angle
float trig_offsetx(uint8_t x)
{
	float temp = float(x);
	temp = fast_atan(temp/obj_distance);
	return temp;
}

//translates the y passed into appropriate angle
float trig_offsety(uint8_t y)
{
	float temp;
	if(y < y_center)
		temp = float(y)-laser_offset;
	else
		temp = float(y)+laser_offset;
	
	temp = fast_atan(temp/obj_distance);
	return temp;
}

