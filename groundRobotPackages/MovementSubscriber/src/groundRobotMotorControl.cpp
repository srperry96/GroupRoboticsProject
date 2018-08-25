/* Functions for calculating and setting the hardware PWM on the Raspberry Pi to
make the ground robot's wheels move.
Written by Samuel Perry */

#include "groundRobotMotorControl.h"

//ID of the raspberry pi to be used in GPIO operations
int pi;

/* Connect to the GPIO daemon, setup PWM pins and ensure the robot is not moving.
The pigpiod daemon must be running on the Pi. In terminal, >sudo pigpiod will start it */
void setupMotors(){
	//connect to pigiod daemon
	pi = pigpio_start(NULL, NULL);

	//setup direction pin 1
	set_mode(pi, 26, PI_OUTPUT);

	//setup direction pin 2
	set_mode(pi, 6, PI_OUTPUT);

	//ensure robot is not moving
	brake();
}

/* Calculate what value of PWM we want to use based on a speed value from -1.0 to 1.0.
Simple calculation to convert to a PWM value from PWMMINLIM to PWMMAXLIM (defined in header) */
int calculatePWM(float speed){
	if(speed < 0) speed = -speed;

	if(speed > 1){
		return PWMMAXLIM;
	}else{
		return ((PWMMAXLIM - PWMMINLIM) * speed) + PWMMINLIM;
	}
}

/* Set the PWM value of a given wheel corresponding to a given speed */
void setPWM(int wheel, float speed){
	//get PWM value based on speed
	unsigned long pwmValue = calculatePWM(speed);

	//set the corresponding wheel PWM
	if(wheel == RIGHTWHEEL){
		//set direction pin
		if(speed < 0){
			gpio_write(pi, 26, 1);
		}else{
			gpio_write(pi, 26, 0);
		}
		//set PWM
		hardware_PWM(pi, 13, 5000, pwmValue);
	}else if(wheel == LEFTWHEEL){
		//set direction pin
		if(speed < 0){
			gpio_write(pi, 6, 0);
		}else{
			gpio_write(pi, 6, 1);
		}
		//set PWM
		hardware_PWM(pi, 12, 5000, pwmValue);
	}
}

/* Set PWM to 0 for both wheels so the robot stops moving */
void brake(){
	hardware_PWM(pi, 12, 0, 0);
	hardware_PWM(pi, 13, 0, 0);
}

/* Close connection to the GPIO daemon - stops hardware pwm output */
void endGPIO(){
	pigpio_stop(pi);
}
