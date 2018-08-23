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

//IR readings
extern uint8_t irValues[8];

/* Sets up the SPI connection to the Arduino */
int setupSPIComms(void);

/* Transmits one byte via SPI, returns a byte from SPI */
int spiTxRx(unsigned char txDat);

/* Check spi is working with a simple handshake */
int spiHandshake(void);

/* Get a set of 8 IR values from the arduino via spi */
void irGetValues(void);

/* Tell the arm to grip in the high position */
void armGripHigh(void);

/* Tell the arm to grip in the low position */
void armGripLow(void);

/* Reset the arm to its initial position */
void armReset(void);

/* Tilt the arm camera down (used when the teddy is below the camera) */
void armTiltCameraLow(void);

/* Tilt the arm camera to its central position */
void armTiltCameraCenter(void);

/* Get a sensor reading from the laser (time of flight) sensor */
uint8_t laserGetReading(void);

#endif
