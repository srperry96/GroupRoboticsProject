/* Functions for using a single time of flight distance sensor.
Written by Samuel Perry (based on an example function by Robert Woolley) */

#ifndef __LASERSENSOR_H__
#define __LASERSENSOR_H__

#include <Wire.h>
#include <VL53L0X.h>

//Most recent laser distance reading
extern uint8_t lastLaserReading;

/* Set up the laser sensor - i2c connection so we use the standard Wire library */
void laserSetup(void);

/* Get a distance reading from the laser time of flight sensor */
void laserGetMeasurement(void);


#endif
