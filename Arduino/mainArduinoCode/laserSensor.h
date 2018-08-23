#ifndef __LASERSENSOR_H__
#define __LASERSENSOR_H__

#include <Wire.h>
#include <VL53L0X.h>

extern uint8_t lastLaserReading;

void laserSetup(void);

void laserGetMeasurement(void);


#endif
