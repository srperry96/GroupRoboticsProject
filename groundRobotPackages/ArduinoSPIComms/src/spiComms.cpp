#include "spiComms.h"

using namespace std;

//spi directory variable
int fd;

uint8_t irValues[8] = {};

/* Sets up the SPI connection to the Arduino */
int setupSPIComms(){
   fd = open("/dev/spidev0.0", O_RDWR);
   unsigned int speed = 1000000;
   ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

   	//Check SPI is working
	if(!spiHandshake()){
		printf("FAILED TO CONNECT SPI\n");
		return 0;
	}else{
		printf("SPI connected\n");
		return 1;
	};
}

/* Transmits one byte via SPI, returns a byte from SPI */
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

/* Check spi is working with a simple handshake */
int spiHandshake(){

	char received;

  //transmit start byte 'a' corresponding to a handshake byte
	spiTxRx('a');
	usleep(10);
  //transmit another 0 so we can receive a byte
	received = spiTxRx(0);

  //if we receive an 'a' back, handshake is successful
	if(received == 'a'){
		printf("SPI connected successfully\n");
		return 1;
	}else{
		printf("ERROR: SPI couldn't connect. Is the Arduino connected?\n");
		return 0;
	}
}

/* Get a set of 8 IR values from the arduino via spi */
void irGetValues(){

    char received = 0;
    bool ack = false;

    do{
      //transmit start byte b
      received = spiTxRx('b');
      usleep(10);

    }while(received != 'b');

    //transmit a byte (doesnt matter what), so we can read a byte
    for(int i = 0; i < 8; i++){
      received = spiTxRx(0);
      irValues[i] = (uint8_t)received;
      //if value is over ~40 the sensors are inaccurate, so set to 127. also if 0 as this indicates a bad read
      if((irValues[i] == 0) || (irValues[i] > 50)){
          irValues[i] = 127;
      }
      usleep (10);
    }
}

/* Tell the arm to grip in the high position */
void armGripHigh(){
  char received;

  spiTxRx('c');
  usleep(10);

  do{
    received = spiTxRx('c');
    usleep(10);
  }while(received != 'c');


  printf("Arm gripped high\n");
}

/* Tell the arm to grip in the low position */
void armGripLow(){
  char received;

  spiTxRx('d');
  usleep(10);

  do{
    received = spiTxRx('d');
    usleep(10);
  }while(received != 'd');

  printf("Arm gripped low\n");
}

/* Reset the arm to its initial position */
void armReset(){
  char received;

  spiTxRx('e');
  usleep(10);

  do{
    received = spiTxRx('e');
    usleep(10);
  }while(received != 'e');

  printf("Arm reset to start position\n");
}

/* Tilt the arm camera down (used when the teddy is below the camera) */
void armTiltCameraLow(){
  char received;

  spiTxRx('f');
  usleep(10);

  do{
    received = spiTxRx('f');
    usleep(10);
  }while(received != 'f');

  printf("Arm camera tilted down\n");
}

void armTiltCameraCenter(){
  char received;

  spiTxRx('g');
  usleep(10);

  do{
    received = spiTxRx('g');
    usleep(10);
  }while(received != 'g');

  printf("Arm camera tilted to center\n");
}

/* Get a sensor reading from the laser (time of flight) sensor */
uint8_t laserGetReading(){
	char received = 0;

  //transmit h and wait for acknowledgement
	do{
		received = spiTxRx('h');
		usleep(10);
	}while(received != 'h');

  //push one more 0 to retrieve our measurement
  received = spiTxRx(0);

	printf("laser scan value %d\n", (uint8_t)received);

  return (uint8_t)received;
}
