/* Functions for calculating and setting the hardware PWM on the Raspberry Pi to
make the ground robot's wheels move.
Written by Samuel Perry */

#ifndef __GROUNDROBOTMOTORCONTROL_H__
#define __GROUNDROBOTMOTORCONTROL_H__

#include <stdio.h>
#include <stdint.h>
#include <pigpiod_if2.h>

#define RIGHTWHEEL 1
#define LEFTWHEEL 2

#define PWMMAXLIM 900000
#define PWMMINLIM 200000

/* Connect to the GPIO daemon, setup PWM pins and ensure the robot is not moving.
The pigpiod daemon must be running on the Pi. In terminal, >sudo pigpiod will start it */
void setupMotors(void);

/* Calculate what value of PWM we want to use based on a speed value from -1.0 to 1.0.
Simple calculation to convert to a PWM value from PWMMINLIM to PWMMAXLIM (defined in header) */
int calculatePWM(float speed);

/* Set the PWM value of a given wheel corresponding to a given speed */
void setPWM(int wheel, float speed);

/* Set PWM to 0 for both wheels so the robot stops moving */
void brake(void);

/* Close connection to the GPIO daemon - stops hardware pwm output */
void endGPIO(void);

#endif
