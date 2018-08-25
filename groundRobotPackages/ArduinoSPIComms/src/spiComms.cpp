/* All functions related to the SPI communication between the Raspberry Pi and the
Arduino in the ground robot.
Written by Samuel Perry (spiTxRx() function and three other lines of code copied
from http://robotics.hobbizine.com/raspiduino.html) */

#include "spiComms.h"

using namespace std;

//spi directory variable
int fd;

uint8_t irValues[8] = {};

/* Sets up the SPI connection to the Arduino */
int setupSPIComms(){
  //the following three lines set up the SPI interface. They are copied directly
  //from an example code at http://robotics.hobbizine.com/raspiduino.html
  fd = open("/dev/spidev0.0", O_RDWR);
  unsigned int speed = 1000000;
  ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

  //Check SPI is working with a simple handshake
	if(!spiHandshake()){
		printf("FAILED TO CONNECT SPI\n");
		return 0;
	}else{
		printf("SPI connected\n");
		return 1;
	};
}

/* Transmits one byte via SPI, returns a byte from SPI. This code is directly copied
from http://robotics.hobbizine.com/raspiduino.html (hence no comments) */
int spiTxRx(unsigned char txDat){

  unsigned char rxDat;

  struct spi_ioc_transfer spi;

  memset (&spi, 0, sizeof(spi));

  spi.tx_buf = (unsigned long)&txDat;
  spi.rx_buf = (unsigned long)&rxDat;
  spi.len = 1;

  ioctl (fd, SPI_IOC_MESSAGE(1), &spi);

  return rxDat;
}

/* NOTE: From here onwards each function follows a similar process for SPI communication.
First, a byte is transmitted, then we wait for an acknowledgement byte. Once received, we
can carry out the rest of the operation. Comments in the following code are only written
for the first occurence of a process so there isn't repetition */
/* The following is the list of characters transmitted and their corresponding
function:
    a - handshake
    b - IR      - get a set of IR readings
    c - gripper - grip high
    d - gripper - grip low
    e - gripper - reset
    f - gripper - tilt camera to low position
    g - gripper - tilt camera to centre position
    h - laser   - get the most recent reading
*/


/* Check SPI communication is working with a simple handshake */
int spiHandshake(){
	char received;

  //transmit start byte 'a' corresponding to a handshake byte
	spiTxRx('a');
	usleep(10);
  //transmit another 0 so we can receive a byte
	received = spiTxRx(0);

  //if we received an 'a' back, handshake is successful
	if(received == 'a'){
		printf("SPI connected successfully\n");
		return 1;
	}else{
		printf("ERROR: SPI couldn't connect. Is the Arduino connected?\n");
		return 0;
	}
}

/* Get a set of 8 IR values from the Arduino via SPI */
void irGetValues(){
    char received;

    do{
      //transmit start byte b
      received = spiTxRx('b');
      usleep(10);
    }while(received != 'b'); //loop until we receive the correct acknowledgement

    //transmit a byte (doesnt matter what), so we can read a byte (this is how Pi SPI communication works)
    for(int i = 0; i < 8; i++){
      //save the received values in the irValues array
      received = spiTxRx(0);
      irValues[i] = (uint8_t)received;
      //if value is over ~50cm the sensors become inaccurate, so set to 127. also if 0 as this indicates a bad read
      if((irValues[i] == 0) || (irValues[i] > 50)){
          irValues[i] = 127;
      }
      //sleep for 10us so SPI transfer has time to occur
      usleep (10);
    }
}

/* Tell the arm to grip in the high position */
void armGripHigh(){
  char received;

  do{
    received = spiTxRx('c');
    usleep(10);
  }while(received != 'c');

  //debugging printout
  printf("Arm gripped high\n");
}

/* Tell the arm to grip in the low position */
void armGripLow(){
  char received;

  do{
    received = spiTxRx('d');
    usleep(10);
  }while(received != 'd');

  printf("Arm gripped low\n");
}

/* Reset the arm to its initial position */
void armReset(){
  char received;

  do{
    received = spiTxRx('e');
    usleep(10);
  }while(received != 'e');

  printf("Arm reset to start position\n");
}

/* Tilt the arm camera down (used when the teddy is below the camera) */
void armTiltCameraLow(){
  char received;

  do{
    received = spiTxRx('f');
    usleep(10);
  }while(received != 'f');

  printf("Arm camera tilted down\n");
}

/* Tilt the arm camera to its central position */
void armTiltCameraCenter(){
  char received;

  do{
    received = spiTxRx('g');
    usleep(10);
  }while(received != 'g');

  printf("Arm camera tilted to center\n");
}

/* Get a sensor reading from the laser (time of flight) sensor */
uint8_t laserGetReading(){
	char received = 0;

	do{
		received = spiTxRx('h');
		usleep(10);
	}while(received != 'h');

  //push one more 0 to retrieve our measurement
  received = spiTxRx(0);

	printf("Laser scan value %d\n", (uint8_t)received);

  return (uint8_t)received;
}
