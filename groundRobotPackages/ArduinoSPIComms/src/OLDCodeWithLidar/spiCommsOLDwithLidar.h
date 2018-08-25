/* An old version of the SPI communications code. Included for completeness as
a significant amount of code was written for use with the LIDAR, which was scrapped
late in the project.
Written by Samuel Perry */

#ifndef __SPICOMMS_H__
#define __SPICOMMS_H__

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

//spi directory variable
extern int fd;

//lidar scan increment
extern uint8_t lastScanIncrement;

//IR readings
extern uint8_t irValues[8];

//full set of lidar data
extern uint8_t fullScanData[360];


/* Sets up the SPI connection to the Arduino */
int setupSPIComms(void);

/* Transmits one byte via SPI, returns a byte from SPI */
int spiTxRx(unsigned char txDat);

/* Check spi is working with a simple handshake */
int spiHandshake(void);

/* Get a set of 8 IR values from the arduino via spi */
void irGetValues(void);

/* Trigger a full scan of lidar at a specific angle increment */
void lidarStartFullScan(uint8_t increment);

/* Request a full set of lidar data (using lastIncrement, so only new data is transmitted) */
void lidarGetFullScanData(void);

/* Set an angle for the lidar servo to move to */
void lidarGoToAngle(uint16_t angle);

/* Trigger a single scan of the lidar (sensorNum defines which sensor to take a reading from) */
uint8_t lidarSingleScan(uint8_t sensorNum);

/* Set the lidar scan range (range is 1-short, 2-long) */
void lidarSetRange(uint8_t range);

/* Set the timing budget used in lidar scans (budget is an 8 bit integer which is multiplied by 1000 on the arduino side) */
void lidarSetTimingBudget(uint8_t budget);

/* Tell the arm to grip in the high position */
void armGripHigh(void);

/* Tell the arm to grip in the low position */
void armGripLow(void);

/* Reset the arm to its initial position */
void armReset(void);

/* Tilt the arm camera down (used when the teddy is below the camera) */
void armTiltCameraLow(void);


#endif
