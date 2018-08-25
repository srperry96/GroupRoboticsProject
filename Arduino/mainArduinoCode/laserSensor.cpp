/* Functions for using a single time of flight distance sensor.
Written by Samuel Perry (based on an example function by Robert Woolley) */
#include "laserSensor.h"

//The time of flight sensor
VL53L0X sensor;

//Most recent laser distance reading
uint8_t lastLaserReading = 0;

/* Set up the laser sensor - i2c connection so we use the standard Wire library */
void laserSetup(){
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
}

/* Get a distance reading from the laser time of flight sensor */
void laserGetMeasurement(){
  lastLaserReading = uint8_t((sensor.readRangeSingleMillimeters()+5)/10);
}
