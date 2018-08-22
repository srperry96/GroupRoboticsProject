#ifndef __IRFUNCS_H__
#define __IRFUNCS_H__

#include <Arduino.h>

//define IR sensor MUX select pins
#define IRS0 6
#define IRS1 7
#define IRS2 8
//define IR sensor input pin
#define IRIN 0

//Array holding the most recent set of IR readings
extern uint8_t irDistances[8];

/* Set up the three MUX select pins as outputs */
void irSetupMultiplexerPins();

/* Set MUX select pins for a given sensor, take a reading and scale to get a distance value in cm from
the voltage level reading */
uint8_t irReadSingleSensor(int sensorNum);

/* Loop through all 8 IR sensors taking readings from each one */
void irReadSensors();

/* Set the IR multiplexer select pins corresponding to the sensor we wish to read */
void irWriteSelectPins(int num);


#endif
