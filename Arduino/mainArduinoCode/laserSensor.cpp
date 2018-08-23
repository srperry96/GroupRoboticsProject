#include "laserSensor.h"

VL53L0X sensor;

uint8_t lastLaserReading = 0;

void laserSetup(){
  delay(1000);
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
}

void laserGetMeasurement(){
  lastLaserReading = uint8_t((sensor.readRangeSingleMillimeters()+5)/10);
}
