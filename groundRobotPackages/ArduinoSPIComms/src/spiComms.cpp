#include "spiComms.h"

using namespace std;

//spi directory variable
int fd;

//lidar scan increment
uint8_t lastScanIncrement = 1;

uint8_t irValues[8] = {};

//full set of lidar data
uint8_t fullScanData[360] = {};

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

	//printout for debugging purposes
//    for(int i = 0; i < 8; i++){
//      printf("Sensor %d: %d;\n", i, irValues[i]);
//    }
}


/* Trigger a full scan of lidar at a specific angle increment */
void lidarStartFullScan(uint8_t increment){
	char received = 0;

	spiTxRx('c');
	usleep(10);

	do{
		received = spiTxRx((char)increment);
		usleep(10);
	}while(received != 'c');

	lastScanIncrement = increment;

	printf("LIDAR SCAN TRIGGERED AT INCREMENT %d. WAIT A BIT AND THEN REQUEST RESULTS\n", lastScanIncrement);
}

/* Request a full set of lidar data (using lastIncrement, so only new data is transmitted) */
void lidarGetFullScanData(){
	char received = 0;

	spiTxRx('d');
	usleep(10);

	do{
		received = spiTxRx('d');
		usleep(10);
	}while(received != 'd');

	for(int i = 0; i < 360; i += lastScanIncrement){
		fullScanData[i] = spiTxRx(0);
	}
	printf("finished receiving scan data at increment %d\n", lastScanIncrement);
}

/* Set an angle for the lidar servo to move to */
void lidarGoToAngle(uint16_t angle){
	char received = 0;

	spiTxRx('e');
	usleep(10);

	do{
		//transmit top 8 bits of angle
		received = spiTxRx((char)((angle & 0xFF00) >> 8));
		usleep(10);
	}while(received != 'e');

	//transmit bottom 8 bits of angle
	spiTxRx((char)(angle & 0x00FF));
	usleep(10);

	do{
		received = spiTxRx(0);
		usleep(10);
	}while(received != 'x');

	printf("Reached angle %d\n", angle);
}

/* Trigger a single scan of the lidar (sensorNum defines which sensor to take a reading from) */
uint8_t lidarSingleScan(uint8_t sensorNum){
	char received = 0;

	//trigger a scan
	spiTxRx('f');
	usleep(10);

	do{
		received = spiTxRx((char)sensorNum);
		usleep(10);
	}while(received != 'f');

	printf("triggered single scan on laser ID %d\n", sensorNum);

	//wait for a bit
	usleep(1000000);

	//now ask for the result
	spiTxRx('g');
	usleep(10);

	do{
		received = spiTxRx(0);
		usleep(10);
	}while(received != 'g');

	received = spiTxRx(0);

	printf("single laser scan value received: %d\n", (uint8_t)received);

	return (uint8_t)received;
}


/* Set the lidar scan range (range is 1-short, 2-long) */
void lidarSetRange(uint8_t range){
	char received = 0;

	spiTxRx('h');
	usleep(10);

	do{
		//send setting value
		received = spiTxRx((char)range);
		usleep(10);
	}while(received != 'h');

	printf("Set lidar range %d\n", range);
}

/* Set the timing budget used in lidar scans (budget is an 8 bit integer which is multiplied by 1000 on the arduino side) */
void lidarSetTimingBudget(uint8_t budget){
	char received;

	spiTxRx('i');
	usleep(10);

	do{
		received = spiTxRx((char)budget);
		usleep(10);
	}while(received != 'i');

	printf("Set lidar timing budget %dx10^3\n", budget);
}

/* Tell the arm to grip in the high position */
void armGripHigh(){
  char received;

  spiTxRx('j');
  usleep(10);

  do{
    received = spiTxRx('j');
    usleep(10);
  }while(received != 'j');


  printf("Arm gripped high\n");
}

/* Tell the arm to grip in the low position */
void armGripLow(){
  char received;

  spiTxRx('k');
  usleep(10);

  do{
    received = spiTxRx('k');
    usleep(10);
  }while(received != 'k');

  printf("Arm gripped low\n");
}

/* Reset the arm to its initial position */
void armReset(){
  char received;

  spiTxRx('l');
  usleep(10);

  do{
    received = spiTxRx('l');
    usleep(10);
  }while(received != 'l');

  printf("Arm reset to start position\n");
}

/* Tilt the arm camera down (used when the teddy is below the camera) */
void armTiltCameraLow(){
  char received;

  spiTxRx('m');
  usleep(10);

  do{
    received = spiTxRx('m');
    usleep(10);
  }while(received != 'm');

  printf("Arm camera tilted down\n");
}
