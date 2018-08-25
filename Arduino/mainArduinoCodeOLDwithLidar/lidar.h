/* Functions to deal with the LIDAR. LIDAR was scrapped late on in the project,
this is included for completeness.
Written by Robert Woolley (modifications by Samuel Perry to make it useable as
a library rather than a standalone piece of arduino code).  */

#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Arduino.h>

// choose pin for PWM data for servo
#define servoPin 5

// new address sensor 2 is set to
#define Sensor2_newAddress 42

// shutdown pins for second laser
#define XSHUT_pin1 9

class Lidar {
  public:
    // creates an instance for the servo and an instance for each of the lasers
    Servo servo;
    VL53L0X sensor1;
    VL53L0X sensor2;
    // where data for the scan will be held
    uint8_t scan [360];
    uint8_t newScan [360];

    //single scan value
    uint8_t singleScanReading = 0;

    bool moving = false;
    unsigned long currentMillis;
    unsigned long startMillis;

    void turnToAngle(uint8_t angle);

    //starting angle of the lidar
    int current_angle = 90;
    void setLongRange (bool long_range);
    void setTimingBudget (long budget);
    void fullScan(uint8_t increment);
    void singleScan(uint8_t sensorNum);

    Lidar();
};


#endif
